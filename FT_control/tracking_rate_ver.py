# ***********************************************************************
#
# 6-axis F/T virtual-point tracking-rate controller for Indy teleoperation.
#
# Control idea:
#   1. Bias-compensated force/torque moves a virtual target pose.
#   2. The robot command pose follows that virtual target using a
#      direct tracking rate:
#
#          x_dot = tracking_rate * (x_virtual - x_command)
#
#   3. MoveTeleL receives the accumulated 6D relative task pose:
#      [x, y, z, Rx, Ry, Rz].
#
# Notes:
#   - Translation units are mm.
#   - Rotation units are assumed to be deg for Indy task poses.
#   - The real TCP pose is approximated by the last commanded relative pose.
#     If robot feedback pose is needed later, replace command_pose with the
#     measured TCP-relative pose in the follow error calculation.
#
# ***********************************************************************

from __future__ import annotations

import json
import logging
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import can
from neuromeka import IndyDCP3

try:
    from neuromeka.proto_step import control_msgs_pb2 as control_msgs
except ModuleNotFoundError:
    from neuromeka.proto import control_msgs_pb2 as control_msgs


log = logging.getLogger('VirtualAdmittance')


def axis_names():
    return ('X(tool)', 'Y(tool)', 'Z(tool)', 'Rx(tool)', 'Ry(tool)', 'Rz(tool)')


def axis_units():
    return ('mm', 'mm', 'mm', 'deg', 'deg', 'deg')


def default_config_path():
    return Path(__file__).resolve().parent / 'config.json'


# ===================================================================
# Configuration
# ===================================================================

@dataclass(frozen=True)
class CanConfig:
    interface: str
    channel: str
    bitrate: int
    force_id: int
    torque_id: int


@dataclass(frozen=True)
class RobotConfig:
    ip: str
    index: int
    apply_robot_commands: bool
    teleop_start_mode: int
    vel_ratio: float
    acc_ratio: float


@dataclass(frozen=True)
class CommonControlConfig:
    period_sec: float
    bias_sample_count: int
    stale_sensor_timeout_sec: float
    force_threshold: float
    release_hold_sec: float


@dataclass(frozen=True)
class ControlConfig:
    common: CommonControlConfig
    tracking_rate: float


@dataclass(frozen=True)
class AppConfig:
    can: CanConfig
    robot: RobotConfig
    control: ControlConfig


def load_config(config_path: Path) -> AppConfig:
    with config_path.open('r', encoding='utf-8') as f:
        data = json.load(f)

    can_cfg = data['can']
    robot_cfg = data['robot']
    control_cfg = data['control']
    common_cfg = control_cfg['common']
    tracking_cfg = control_cfg['tracking_rate']

    config = AppConfig(
        can=CanConfig(
            interface=can_cfg['interface'],
            channel=can_cfg['channel'],
            bitrate=int(can_cfg['bitrate']),
            force_id=int(can_cfg['force_id']),
            torque_id=int(can_cfg['torque_id']),
        ),
        robot=RobotConfig(
            ip=robot_cfg['ip'],
            index=int(robot_cfg['index']),
            apply_robot_commands=bool(robot_cfg['apply_robot_commands']),
            teleop_start_mode=int(robot_cfg['teleop_start_mode']),
            vel_ratio=float(robot_cfg['vel_ratio']),
            acc_ratio=float(robot_cfg['acc_ratio']),
        ),
        control=ControlConfig(
            common=CommonControlConfig(
                period_sec=float(common_cfg['period_sec']),
                bias_sample_count=int(common_cfg['bias_sample_count']),
                stale_sensor_timeout_sec=float(common_cfg['stale_sensor_timeout_sec']),
                force_threshold=float(common_cfg['force_threshold']),
                release_hold_sec=float(common_cfg['release_hold_sec']),
            ),
            tracking_rate=float(tracking_cfg['tracking_rate']),
        ),
    )
    if config.control.common.bias_sample_count <= 0:
        raise ValueError('control.common.bias_sample_count must be positive')
    if config.control.common.period_sec <= 0.0:
        raise ValueError('control.common.period_sec must be positive')
    if config.control.common.stale_sensor_timeout_sec <= 0.0:
        raise ValueError('control.common.stale_sensor_timeout_sec must be positive')
    if config.control.common.force_threshold < 0.0:
        raise ValueError('control.common.force_threshold must be non-negative')
    if config.control.common.release_hold_sec < 0.0:
        raise ValueError('control.common.release_hold_sec must be non-negative')
    if config.control.tracking_rate <= 0.0:
        raise ValueError('control.tracking_rate.tracking_rate must be positive')
    return config




def resolve_config_path(argv: Sequence[str]) -> Path:
    if not argv:
        return default_config_path()
    if argv[0] in ('-c', '--config'):
        if len(argv) < 2:
            raise ValueError('missing config path after -c/--config')
        return Path(argv[1]).expanduser()
    return Path(argv[0]).expanduser()


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
        handlers=[logging.StreamHandler(sys.stdout)],
        # force=True,
    )


# ===================================================================
# F/T sensor reader
# ===================================================================

class FTSensorReader(object):
    def __init__(self, config: CanConfig):
        self._interface = config.interface
        self._channel = config.channel
        self._bitrate = config.bitrate
        self._force_id = config.force_id
        self._torque_id = config.torque_id
        self._lock = threading.Lock()
        self._Fx = self._Fy = self._Fz = self._Tx = self._Ty = self._Tz = 0.0
        self._last_sample_time: Optional[float] = None
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
                if len(msg.data) < 6:
                    log.warning('[CAN] ignored short frame: id=0x%03X len=%d',
                                msg.arbitration_id, len(msg.data))
                    continue

                now = time.monotonic()
                if msg.arbitration_id == self._force_id:
                    d = msg.data
                    with self._lock:
                        self._Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
                        self._Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
                        self._Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
                        self._last_sample_time = now
                    self._new_data.set()
                elif msg.arbitration_id == self._torque_id:
                    d = msg.data
                    with self._lock:
                        self._Tx = (d[0] * 256 + d[1]) / 500.0 - 50.0
                        self._Ty = (d[2] * 256 + d[3]) / 500.0 - 50.0
                        self._Tz = (d[4] * 256 + d[5]) / 500.0 - 50.0
                        self._last_sample_time = now
                    self._new_data.set()
            except Exception as e:
                if self._running:
                    log.error('[CAN] receive error: %s', e)

    def get_ft(self):
        with self._lock:
            return [self._Fx, self._Fy, self._Fz,
                    self._Tx, self._Ty, self._Tz]

    def sample_age_sec(self) -> Optional[float]:
        with self._lock:
            if self._last_sample_time is None:
                return None
            last_sample_time = self._last_sample_time
        return time.monotonic() - last_sample_time

    def is_fresh(self, timeout_sec: float) -> bool:
        age = self.sample_age_sec()
        return age is not None and age <= timeout_sec

    def wait_for_data(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._last_sample_time is not None:
                    return True
            remaining = max(0.0, deadline - time.monotonic())
            self._new_data.wait(timeout=min(0.05, remaining))
            self._new_data.clear()
        return False


# ===================================================================
# Control helpers
# ===================================================================


def measure_bias(sensor, sample_count: int, period_sec: float, stale_timeout_sec: float):
    log.info('Bias measurement: collecting %d samples. Keep robot and sensor still.',
             sample_count)
    accum = [0.0] * 6
    for _ in range(sample_count):
        sample_age = sensor.sample_age_sec()
        if sample_age is None or sample_age > stale_timeout_sec:
            age_msg = 'unknown' if sample_age is None else '{:.1f}ms'.format(sample_age * 1000)
            raise RuntimeError('F/T sensor stale during bias measurement: {}'.format(age_msg))

        ft = sensor.get_ft()
        for j in range(6):
            accum[j] += ft[j]
        time.sleep(period_sec)

    bias = [accum[j] / sample_count for j in range(6)]
    log.info('Bias: F=[%.3f, %.3f, %.3f]N, T=[%.3f, %.3f, %.3f]Nm',
             bias[0], bias[1], bias[2], bias[3], bias[4], bias[5])
    return bias


def compensate_bias(ft_raw, bias):
    return [ft_raw[i] - bias[i] for i in range(6)]


def update_force_activity(ft_comp, force_threshold, last_force_time, now, release_hold_sec):
    force_detected = max(abs(ft_comp[i]) for i in range(3)) > force_threshold
    if force_detected:
        return True, True, now

    if last_force_time is None:
        return False, False, last_force_time

    force_recent = (now - last_force_time) <= release_hold_sec
    return force_detected, force_recent, last_force_time


def update_virtual_target(
    virtual_pose,
    ft_comp,
    dt,
):
    virtual_step = [
        ft_comp[0] * dt,
        ft_comp[1] * dt,
        ft_comp[2] * dt,
        ft_comp[3] * dt,
        ft_comp[4] * dt,
        ft_comp[5] * dt,
    ]

    for i in range(6):
        virtual_pose[i] += virtual_step[i]

    return virtual_step


def compute_follow_step(
    virtual_pose,
    command_pose,
    dt,
    control_config: ControlConfig,
):
    error = [virtual_pose[i] - command_pose[i] for i in range(6)]
    command_step = [control_config.tracking_rate * error[i] * dt for i in range(6)]

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


def log_status(ft_raw, ft_comp, virtual_step, command_step,
               virtual_pose, command_pose, error, loop_count):
    if loop_count % 10 != 0:
        return

    moving_axes = []
    units = axis_units()
    for i, name in enumerate(axis_names()):
        if abs(command_step[i]) > 1e-4:
            direction = '+' if command_step[i] > 0 else '-'
            moving_axes.append('{}{} {:.3f}{}'.format(
                direction, name, abs(command_step[i]), units[i]
            ))
    move_str = ', '.join(moving_axes) if moving_axes else 'stop'

    log.debug(
        '[Loop %4d] RAW F=[%+6.2f,%+6.2f,%+6.2f]N RAW T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[Loop %4d] COMP F=[%+6.2f,%+6.2f,%+6.2f]N COMP T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
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


def log_config_summary(config: AppConfig, config_path: Path) -> None:
    log.info('=' * 70)
    log.info('6-axis virtual-point controller started')
    log.info('Config: %s', config_path)
    log.info('Robot command mode: %s',
             'APPLY' if config.robot.apply_robot_commands else 'DEBUG_ONLY')
    log.info('Virtual target input scale: direct F/T integration')
    log.info('Follow mode: tracking_rate')
    log.info('Tracking rate=%.3f 1/s', config.control.tracking_rate)
    log.info('Force threshold=%.3fN, release hold=%.3fs',
             config.control.common.force_threshold,
             config.control.common.release_hold_sec)
    log.info('Sensor stale timeout=%.3fs', config.control.common.stale_sensor_timeout_sec)
    log.info('=' * 70)


# ===================================================================
# Main control loop
# ===================================================================

def main(argv: Optional[Sequence[str]] = None):
    argv = sys.argv[1:] if argv is None else argv
    try:
        config_path = resolve_config_path(argv)
        config = load_config(config_path)
    except Exception as e:
        setup_logging()
        log.error('Configuration load failed: %s', e)
        return

    setup_logging()
    log_config_summary(config, config_path)

    sensor = FTSensorReader(config.can)
    try:
        sensor.start()
    except Exception as e:
        log.error('F/T sensor initialization failed: %s', e)
        return

    log.info('Waiting for first complete F/T sample...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T sensor data timeout')
        sensor.stop()
        return
    log.info('F/T sensor data confirmed')

    indy = None
    if config.robot.apply_robot_commands:
        log.info('Connecting robot: IP=%s, index=%d', config.robot.ip, config.robot.index)
        try:
            indy = IndyDCP3(robot_ip=config.robot.ip, index=config.robot.index)
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
    try:
        bias = measure_bias(
            sensor,
            config.control.common.bias_sample_count,
            config.control.common.period_sec,
            config.control.common.stale_sensor_timeout_sec,
        )
    except RuntimeError as e:
        log.error('%s', e)
        sensor.stop()
        return

    if config.robot.apply_robot_commands:
        log.info('Starting teleoperation mode')
        try:
            indy.start_teleop(config.robot.teleop_start_mode)
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
    last_force_time = None
    prev_time = time.monotonic()

    log.info('Control loop started. Press Ctrl+C to stop.')

    try:
        while True:
            t_start = time.monotonic()
            dt = t_start - prev_time
            prev_time = t_start

            sample_age = sensor.sample_age_sec()
            if sample_age is None or sample_age > config.control.common.stale_sensor_timeout_sec:
                age_msg = 'unknown' if sample_age is None else '{:.1f}ms'.format(sample_age * 1000)
                log.error('F/T sensor stale for %s; stopping control loop', age_msg)
                break

            ft_raw = sensor.get_ft()
            ft_comp = compensate_bias(ft_raw, bias)
            force_detected, force_recent, last_force_time = update_force_activity(
                ft_comp,
                config.control.common.force_threshold,
                last_force_time,
                t_start,
                config.control.common.release_hold_sec,
            )

            if force_detected:
                virtual_step = update_virtual_target(
                    virtual_pose,
                    ft_comp,
                    dt,
                )
            else:
                virtual_step = [0.0] * 6

            if force_recent:
                command_step, error = compute_follow_step(
                    virtual_pose,
                    command_pose,
                    dt,
                    config.control,
                )
            else:
                virtual_pose[:] = command_pose[:]
                command_step = [0.0] * 6
                error = [0.0] * 6

            for i in range(6):
                command_pose[i] += command_step[i]

            log_status(
                ft_raw, ft_comp, virtual_step, command_step,
                virtual_pose, command_pose, error, loop_count
            )

            if config.robot.apply_robot_commands:
                try:
                    indy.control.MoveTeleL(
                        control_msgs.MoveTeleLReq(
                            tpos=command_pose,
                            vel_ratio=config.robot.vel_ratio,
                            acc_ratio=config.robot.acc_ratio,
                            method=control_msgs.TELE_TASK_TCP,
                        )
                    )
                except Exception as e:
                    log.error('MoveTeleL(TCP) error: %s', e)
                    if not check_robot_connection(indy):
                        log.error('Robot connection abnormal; stopping control loop')
                        break

            loop_count += 1

            elapsed = time.monotonic() - t_start
            sleep_time = config.control.common.period_sec - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning('[Loop %d] control period overrun: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, config.control.common.period_sec * 1000)

    except KeyboardInterrupt:
        log.info('Interrupted by user')
    except Exception as e:
        log.error('Control loop exception: %s', e, exc_info=True)
    finally:
        if config.robot.apply_robot_commands and indy is not None:
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
