# ***********************************************************************
#
# 6-axis F/T virtual-point admittance controller for Indy teleoperation.
#
# Control idea:
#   1. Bias-compensated force/torque moves a virtual target pose.
#   2. The robot command pose follows that virtual target through a
#      spring-damper relation without a mass term:
#
#          D * x_dot = K * (x_virtual - x_command)
#
#   3. MoveTeleL receives the accumulated 6D relative task pose:
#      [x, y, z, Rx, Ry, Rz].
#
# Notes:
#   - Translation units are mm.
#   - Rotation units are assumed to be deg for Indy task poses.
#   - The real TCP pose is approximated by the last commanded relative pose.
#     If robot feedback pose is needed later, replace command_pose with the
#     measured TCP-relative pose in the spring-damper error calculation.
#
# ***********************************************************************

from __future__ import annotations

import logging
import sys
import threading
import time
from typing import List, Tuple

import can
from neuromeka import IndyDCP3

try:
    from neuromeka.proto_step import control_msgs_pb2 as control_msgs
except ModuleNotFoundError:
    from neuromeka.proto import control_msgs_pb2 as control_msgs


# ===================================================================
# Hardware settings
# ===================================================================

CAN_INTERFACE = 'slcan'
CAN_CHANNEL = 'COM3'
CAN_BITRATE = 1_000_000

CAN_ID_FORCE = 0x001
CAN_ID_TORQUE = 0x002

ROBOT_IP = '192.168.0.137'
ROBOT_INDEX = 0

# Switch this single value to move between dry-run debugging and real control.
# False: print F/T, virtual target, computed command direction only.
# True: enter teleop mode and send MoveTeleL commands to the robot.
APPLY_ROBOT_COMMANDS = True


# ===================================================================
# Control parameters
# ===================================================================

BIAS_SAMPLE_COUNT = 200
BIAS_SAMPLE_DELAY = 0.005

# Input deadbands.
FORCE_THRESHOLD_XY = 0.5       # N
FORCE_THRESHOLD_Z = 1.0        # N
TORQUE_THRESHOLD_RXRY = 0.05   # Nm
TORQUE_THRESHOLD_RZ = 0.05     # Nm

# How fast the operator's wrench moves the virtual target.
# This is the "handle sensitivity" before the robot follows the target.
VIRTUAL_POINT_FORCE_GAIN_XY = 2.0       # mm / (N*s)
VIRTUAL_POINT_FORCE_GAIN_Z = 1.5        # mm / (N*s)
VIRTUAL_POINT_TORQUE_GAIN_RXRY = 5.0    # deg / (Nm*s)
VIRTUAL_POINT_TORQUE_GAIN_RZ = 5.0      # deg / (Nm*s)

# Spring-damper follow dynamics: D * x_dot = K * error.
# Higher K/D ratio follows the virtual target faster.
STIFFNESS_XY = 1.0           # N/mm
STIFFNESS_Z = 1.0            # N/mm
DAMPING_XY = 0.25            # N*s/mm
DAMPING_Z = 0.25             # N*s/mm

ROT_STIFFNESS_RXRY = 0.10    # Nm/deg
ROT_STIFFNESS_RZ = 0.10      # Nm/deg
ROT_DAMPING_RXRY = 0.05      # Nm*s/deg
ROT_DAMPING_RZ = 0.05        # Nm*s/deg

# Safety clamps per control loop.
MAX_VIRTUAL_STEP_MM = 5.0
MAX_VIRTUAL_STEP_DEG = 1.0
MAX_COMMAND_STEP_MM = 2.0
MAX_COMMAND_STEP_DEG = 0.25

TEL_VEL_RATIO = 0.5
TEL_ACC_RATIO = 1.0

CONTROL_PERIOD = 0.02
MAX_DT = CONTROL_PERIOD * 2

AXIS_NAMES = ['X(tool)', 'Y(tool)', 'Z(tool)', 'Rx(tool)', 'Ry(tool)', 'Rz(tool)']
AXIS_UNITS = ['mm', 'mm', 'mm', 'deg', 'deg', 'deg']


# ===================================================================
# Logging
# ===================================================================

logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger('VirtualAdmittance')


# ===================================================================
# F/T sensor reader
# ===================================================================

class FTSensorReader(object):
    def __init__(self, interface, channel, bitrate):
        self._interface = interface
        self._channel = channel
        self._bitrate = bitrate
        self._lock = threading.Lock()
        self._Fx = self._Fy = self._Fz = 0.0
        self._Tx = self._Ty = self._Tz = 0.0
        self._running = False
        self._thread = None
        self._bus = None
        self._new_data = threading.Event()

    def start(self):
        log.info('Initializing CAN bus: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._send_start_command()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, name='FTReader', daemon=True)
        self._thread.start()
        log.info('F/T sensor receiver started')

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._bus is not None:
            self._bus.shutdown()
        log.info('F/T sensor receiver stopped')

    def _send_start_command(self):
        data = [0x04, 0x02, 0x06, 0x01, 0x03, 0x01]
        cmd = can.Message(arbitration_id=0x000, data=data, is_extended_id=False)
        try:
            self._bus.send(cmd)
            log.debug('Sensor start command sent')
        except can.CanError as e:
            log.warning('Sensor start command failed, ignored if streaming already: %s', e)

    def _recv_loop(self):
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None:
                    log.warning('[CAN] receive timeout - check sensor connection')
                    continue

                if msg.arbitration_id == CAN_ID_FORCE:
                    d = msg.data
                    with self._lock:
                        self._Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
                        self._Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
                        self._Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
                    self._new_data.set()
                elif msg.arbitration_id == CAN_ID_TORQUE:
                    d = msg.data
                    with self._lock:
                        self._Tx = (d[0] * 256 + d[1]) / 500.0 - 50.0
                        self._Ty = (d[2] * 256 + d[3]) / 500.0 - 50.0
                        self._Tz = (d[4] * 256 + d[5]) / 500.0 - 50.0
                    self._new_data.set()
            except Exception as e:
                if self._running:
                    log.error('[CAN] receive error: %s', e)

    def get_ft(self):
        with self._lock:
            return [self._Fx, self._Fy, self._Fz,
                    self._Tx, self._Ty, self._Tz]

    def wait_for_data(self, timeout=5.0):
        return self._new_data.wait(timeout=timeout)


# ===================================================================
# Control helpers
# ===================================================================

def clip(val, limit):
    return max(-limit, min(limit, val))


def deadband(val, threshold):
    if abs(val) < threshold:
        return 0.0
    return (val - threshold) if val > 0 else (val + threshold)


def measure_bias(sensor, n_samples=BIAS_SAMPLE_COUNT, delay=BIAS_SAMPLE_DELAY):
    log.info('Bias measurement: collecting %d samples. Keep robot and sensor still.', n_samples)
    accum = [0.0] * 6
    for _ in range(n_samples):
        ft = sensor.get_ft()
        for j in range(6):
            accum[j] += ft[j]
        time.sleep(delay)

    bias = [accum[j] / n_samples for j in range(6)]
    log.info('Bias: F=[%.3f, %.3f, %.3f]N, T=[%.3f, %.3f, %.3f]Nm',
             bias[0], bias[1], bias[2], bias[3], bias[4], bias[5])
    return bias


def compensate_and_deadband(ft_raw, bias):
    ft_comp = [ft_raw[i] - bias[i] for i in range(6)]
    wrench_eff = [
        deadband(ft_comp[0], FORCE_THRESHOLD_XY),
        deadband(ft_comp[1], FORCE_THRESHOLD_XY),
        deadband(ft_comp[2], FORCE_THRESHOLD_Z),
        deadband(ft_comp[3], TORQUE_THRESHOLD_RXRY),
        deadband(ft_comp[4], TORQUE_THRESHOLD_RXRY),
        deadband(ft_comp[5], TORQUE_THRESHOLD_RZ),
    ]
    return ft_comp, wrench_eff


def update_virtual_target(virtual_pose, wrench_eff, dt):
    # type: (List[float], List[float], float) -> List[float]
    virtual_step = [
        wrench_eff[0] * VIRTUAL_POINT_FORCE_GAIN_XY * dt,
        wrench_eff[1] * VIRTUAL_POINT_FORCE_GAIN_XY * dt,
        wrench_eff[2] * VIRTUAL_POINT_FORCE_GAIN_Z * dt,
        wrench_eff[3] * VIRTUAL_POINT_TORQUE_GAIN_RXRY * dt,
        wrench_eff[4] * VIRTUAL_POINT_TORQUE_GAIN_RXRY * dt,
        wrench_eff[5] * VIRTUAL_POINT_TORQUE_GAIN_RZ * dt,
    ]

    for i in range(3):
        virtual_step[i] = clip(virtual_step[i], MAX_VIRTUAL_STEP_MM)
    for i in range(3, 6):
        virtual_step[i] = clip(virtual_step[i], MAX_VIRTUAL_STEP_DEG)

    for i in range(6):
        virtual_pose[i] += virtual_step[i]

    return virtual_step


def compute_spring_damper_step(virtual_pose, command_pose, dt):
    # type: (List[float], List[float], float) -> Tuple[List[float], List[float]]
    error = [virtual_pose[i] - command_pose[i] for i in range(6)]

    gains = [
        STIFFNESS_XY / DAMPING_XY,
        STIFFNESS_XY / DAMPING_XY,
        STIFFNESS_Z / DAMPING_Z,
        ROT_STIFFNESS_RXRY / ROT_DAMPING_RXRY,
        ROT_STIFFNESS_RXRY / ROT_DAMPING_RXRY,
        ROT_STIFFNESS_RZ / ROT_DAMPING_RZ,
    ]
    command_step = [gains[i] * error[i] * dt for i in range(6)]

    for i in range(3):
        command_step[i] = clip(command_step[i], MAX_COMMAND_STEP_MM)
    for i in range(3, 6):
        command_step[i] = clip(command_step[i], MAX_COMMAND_STEP_DEG)

    return command_step, error


def check_robot_connection(indy):
    try:
        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        sim_mode = robot_data.get('sim_mode', False)
        log.info('Robot state: op_state=%d, sim_mode=%s', op_state, sim_mode)

        abnormal = {0, 2, 3, 8, 15}
        if op_state in abnormal:
            log.error('Robot is in abnormal state: op_state=%d', op_state)
            return False
        if sim_mode:
            log.warning('Simulation mode is active; real robot may not move.')
        return True
    except Exception as e:
        log.error('Robot connection check failed: %s', e)
        return False


def log_status(ft_raw, ft_comp, wrench_eff, virtual_step, command_step,
               virtual_pose, command_pose, error, loop_count):
    if loop_count % 10 != 0:
        return

    moving_axes = []
    for i, name in enumerate(AXIS_NAMES):
        if abs(command_step[i]) > 1e-4:
            direction = '+' if command_step[i] > 0 else '-'
            moving_axes.append('{}{} {:.3f}{}'.format(
                direction, name, abs(command_step[i]), AXIS_UNITS[i]
            ))
    move_str = ', '.join(moving_axes) if moving_axes else 'stop'

    log.debug(
        '[Loop %4d] RAW F=[%+6.2f,%+6.2f,%+6.2f]N RAW T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[Loop %4d] COMP F=[%+6.2f,%+6.2f,%+6.2f]N COMP T=[%+6.3f,%+6.3f,%+6.3f]Nm '
        'EFF=[%+6.2f,%+6.2f,%+6.2f,%+6.3f,%+6.3f,%+6.3f]',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
        wrench_eff[0], wrench_eff[1], wrench_eff[2],
        wrench_eff[3], wrench_eff[4], wrench_eff[5],
    )
    log.debug(
        '[Loop %4d] virtual_step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '
        'error=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg',
        loop_count,
        virtual_step[0], virtual_step[1], virtual_step[2],
        virtual_step[3], virtual_step[4], virtual_step[5],
        error[0], error[1], error[2], error[3], error[4], error[5],
    )
    log.debug(
        '[Loop %4d] cmd_step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '
        'cmd_pose=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg axes=%s',
        loop_count,
        command_step[0], command_step[1], command_step[2],
        command_step[3], command_step[4], command_step[5],
        command_pose[0], command_pose[1], command_pose[2],
        command_pose[3], command_pose[4], command_pose[5],
        move_str,
    )
    log.debug(
        '[Loop %4d] virtual_pose=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg',
        loop_count,
        virtual_pose[0], virtual_pose[1], virtual_pose[2],
        virtual_pose[3], virtual_pose[4], virtual_pose[5],
    )


# ===================================================================
# Main control loop
# ===================================================================

def main():
    log.info('=' * 70)
    log.info('6-axis virtual-point admittance controller started')
    log.info('Robot command mode: %s', 'APPLY' if APPLY_ROBOT_COMMANDS else 'DEBUG_ONLY')
    log.info('Model: virtual target from F/T, follow with D*x_dot = K*(x_virtual - x_command)')
    log.info('Input gain F xy/z=%.2f/%.2f mm/(N*s), T rxry/rz=%.2f/%.2f deg/(Nm*s)',
             VIRTUAL_POINT_FORCE_GAIN_XY, VIRTUAL_POINT_FORCE_GAIN_Z,
             VIRTUAL_POINT_TORQUE_GAIN_RXRY, VIRTUAL_POINT_TORQUE_GAIN_RZ)
    log.info('K trans xy/z=%.2f/%.2f N/mm, D trans xy/z=%.2f/%.2f N*s/mm',
             STIFFNESS_XY, STIFFNESS_Z, DAMPING_XY, DAMPING_Z)
    log.info('K rot rxry/rz=%.3f/%.3f Nm/deg, D rot rxry/rz=%.3f/%.3f Nm*s/deg',
             ROT_STIFFNESS_RXRY, ROT_STIFFNESS_RZ, ROT_DAMPING_RXRY, ROT_DAMPING_RZ)
    log.info('=' * 70)

    sensor = FTSensorReader(CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE)
    try:
        sensor.start()
    except Exception as e:
        log.error('F/T sensor initialization failed: %s', e)
        return

    log.info('Waiting for first F/T sample...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T sensor data timeout')
        sensor.stop()
        return
    log.info('F/T sensor data confirmed')

    indy = None
    if APPLY_ROBOT_COMMANDS:
        log.info('Connecting robot: IP=%s, index=%d', ROBOT_IP, ROBOT_INDEX)
        try:
            indy = IndyDCP3(robot_ip=ROBOT_IP, index=ROBOT_INDEX)
        except Exception as e:
            log.error('Robot connection failed: %s', e)
            sensor.stop()
            return

        if not check_robot_connection(indy):
            sensor.stop()
            return
    else:
        log.warning('DEBUG_ONLY mode: robot connection, teleop, and MoveTeleL commands are disabled.')

    input('\nKeep robot and F/T sensor still, then press Enter to measure bias... ')
    bias = measure_bias(sensor)

    if APPLY_ROBOT_COMMANDS:
        log.info('Starting teleoperation mode')
        try:
            indy.start_teleop(2)
            time.sleep(0.5)
        except Exception as e:
            log.error('start_teleop failed: %s', e)
            sensor.stop()
            return

        teleop_state = indy.get_teleop_state()
        log.info('Teleop state: %s', teleop_state)

        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        if op_state != 17:
            log.error('Teleop transition failed: op_state=%d, expected 17=TELE_OP', op_state)
            indy.stop_teleop()
            sensor.stop()
            return

    loop_count = 0
    virtual_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    command_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    prev_time = time.time()

    log.info('Control loop started. Press Ctrl+C to stop.')

    try:
        while True:
            t_start = time.time()
            dt = t_start - prev_time
            prev_time = t_start

            if dt > MAX_DT:
                log.debug('[Loop %d] dt spike %.1fms; using %.0fms',
                          loop_count, dt * 1000, CONTROL_PERIOD * 1000)
                dt = CONTROL_PERIOD

            ft_raw = sensor.get_ft()
            ft_comp, wrench_eff = compensate_and_deadband(ft_raw, bias)

            virtual_step = update_virtual_target(virtual_pose, wrench_eff, dt)
            command_step, error = compute_spring_damper_step(virtual_pose, command_pose, dt)

            for i in range(6):
                command_pose[i] += command_step[i]

            log_status(
                ft_raw, ft_comp, wrench_eff, virtual_step, command_step,
                virtual_pose, command_pose, error, loop_count
            )

            if APPLY_ROBOT_COMMANDS:
                try:
                    indy.control.MoveTeleL(
                        control_msgs.MoveTeleLReq(
                            tpos=command_pose,
                            vel_ratio=TEL_VEL_RATIO,
                            acc_ratio=TEL_ACC_RATIO,
                            method=control_msgs.TELE_TASK_TCP,
                        )
                    )
                except Exception as e:
                    log.error('MoveTeleL(TCP) error: %s', e)
                    if not check_robot_connection(indy):
                        log.error('Robot connection abnormal; stopping control loop')
                        break

            loop_count += 1

            elapsed = time.time() - t_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning('[Loop %d] control period overrun: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, CONTROL_PERIOD * 1000)

    except KeyboardInterrupt:
        log.info('Interrupted by user')
    except Exception as e:
        log.error('Control loop exception: %s', e, exc_info=True)
    finally:
        if APPLY_ROBOT_COMMANDS and indy is not None:
            log.info('Stopping teleoperation...')
            try:
                indy.stop_teleop()
                time.sleep(0.3)
                log.info('Teleoperation stopped')
            except Exception as e:
                log.error('stop_teleop error: %s', e)
        else:
            log.info('DEBUG_ONLY mode: no teleoperation session to stop.')

        sensor.stop()
        log.info('System stopped. Total loops: %d', loop_count)
        log.info('Final virtual pose: X=%.2f Y=%.2f Z=%.2f mm, Rx=%.2f Ry=%.2f Rz=%.2f deg',
                 virtual_pose[0], virtual_pose[1], virtual_pose[2],
                 virtual_pose[3], virtual_pose[4], virtual_pose[5])
        log.info('Final command pose: X=%.2f Y=%.2f Z=%.2f mm, Rx=%.2f Ry=%.2f Rz=%.2f deg',
                 command_pose[0], command_pose[1], command_pose[2],
                 command_pose[3], command_pose[4], command_pose[5])


if __name__ == '__main__':
    main()
