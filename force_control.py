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
# [MODIFIED] Changes from original:
#   - 2-Stage Hysteresis Threshold:
#       * ENGAGE threshold (start): larger force required to begin motion
#       * RELEASE threshold (sustain): smaller force enough to keep moving
#       * Per-axis motion state tracked via 'axis_active' flags
#   - Improved responsiveness:
#       * VIRTUAL_POINT_FORCE_GAIN_XY/Z raised (faster virtual target movement)
#       * K/D ratio raised (robot follows virtual target faster)
#       * MAX_COMMAND_STEP_MM/DEG raised (larger per-loop movement allowed)
#       * TEL_VEL_RATIO raised (robot executes commands faster)
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

# Axis isolation for safer staged tests.
# Options:
#   'X'   -> use Fx and Tx only
#   'Y'   -> use Fy and Ty only
#   'Z'   -> use Fz and Tz only
#   'ALL' -> use all force/torque axes
AXIS_TEST_MODE = 'ALL'


# ===================================================================
# Control parameters
# ===================================================================

BIAS_SAMPLE_COUNT = 200
BIAS_SAMPLE_DELAY = 0.005

# ---------------------------------------------------------------------------
# [MODIFIED] 2-Stage Hysteresis Thresholds
#
# ENGAGE  (start threshold): 정지 상태에서 움직임을 시작하려면 이 값 이상의 힘 필요
# RELEASE (sustain threshold): 한 번 움직이기 시작한 뒤 운동을 유지하기 위한 최소 힘
#
# 동작 원리:
#   axis_active[i] == False (정지):  |force| >= ENGAGE  => axis_active = True, 운동 시작
#   axis_active[i] == True  (운동):  |force| >= RELEASE => 운동 유지
#                                    |force| <  RELEASE => axis_active = False, 정지
#
# 튜닝 가이드:
#   - ENGAGE를 낮추면 작은 힘으로도 쉽게 시작됨 (민감도 ↑, 의도치 않은 움직임 위험 ↑)
#   - ENGAGE를 높이면 확실한 의도가 있어야만 시작됨 (안정성 ↑, 반응성 ↓)
#   - RELEASE를 ENGAGE 대비 40~60% 수준으로 유지하면 히스테리시스 효과가 자연스러움
# ---------------------------------------------------------------------------
FORCE_ENGAGE_XY   = 1.0    # N  — 정지 → 운동 전환에 필요한 힘 (원래 FORCE_THRESHOLD_XY=0.5)
FORCE_RELEASE_XY  = 0.8    # N  — 운동 → 정지 전환 경계 (ENGAGE의 50%)

FORCE_ENGAGE_Z    = 1.0    # N
FORCE_RELEASE_Z   = 0.8    # N

TORQUE_ENGAGE_RXRY   = 0.15  # Nm
TORQUE_RELEASE_RXRY  = 0.07  # Nm

TORQUE_ENGAGE_RZ     = 0.15  # Nm
TORQUE_RELEASE_RZ    = 0.07  # Nm

# ---------------------------------------------------------------------------
# [MODIFIED] Virtual target gain — 힘 입력이 virtual target을 얼마나 빠르게 이동시키는가
#
# 원래 값: XY=4.0, Z=3.0, RXRY=5.0, RZ=5.0
#
# 튜닝 가이드:
#   - 값을 높일수록 같은 힘 입력에 대해 virtual target이 더 많이 이동 → 로봇이 더 크게 반응
#   - 너무 높으면 MAX_VIRTUAL_STEP 클램프에 항상 걸려 실질적 효과 없음
#   - MAX_VIRTUAL_STEP_MM = gain * max_force * dt 가 클램프 이하가 되도록 설정
#     예) gain=8, max_expected_force=5N, dt=0.02 → 0.8mm/loop < 5mm ✓
# ---------------------------------------------------------------------------
VIRTUAL_POINT_FORCE_GAIN_XY    = 8.0   # mm / (N*s)   [원래: 4.0]
VIRTUAL_POINT_FORCE_GAIN_Z     = 6.0   # mm / (N*s)   [원래: 3.0]
VIRTUAL_POINT_TORQUE_GAIN_RXRY = 10.0  # deg / (Nm*s) [원래: 5.0]
VIRTUAL_POINT_TORQUE_GAIN_RZ   = 10.0  # deg / (Nm*s) [원래: 5.0]

# ---------------------------------------------------------------------------
# [MODIFIED] Spring-damper follow dynamics: D * x_dot = K * error
#
# 실질 추종 이득 = K / D (단위: 1/s)
#
# 원래:  K=2.0, D=0.25  => K/D = 8  /s
# 변경:  K=4.0, D=0.20  => K/D = 20 /s  (약 2.5배 빠른 추종)
#
# 튜닝 가이드:
#   - K/D 비율이 높을수록 virtual target을 빠르게 추종 (반응성 ↑)
#   - D를 너무 낮추면 오버슈트 발생 가능
#   - 권장: K/D = 10~25 범위에서 테스트
#
# command_step per loop = (K/D) * error * dt
#   예) K/D=20, error=2mm, dt=0.02s → step=0.8mm/loop
# ---------------------------------------------------------------------------
STIFFNESS_XY  = 4.0   # N/mm     [원래: 2.0]
STIFFNESS_Z   = 4.0   # N/mm     [원래: 2.0]
DAMPING_XY    = 0.20  # N*s/mm   [원래: 0.25]  => K/D = 20/s
DAMPING_Z     = 0.20  # N*s/mm   [원래: 0.25]  => K/D = 20/s

ROT_STIFFNESS_RXRY = 1.0   # Nm/deg     [원래: 0.5]
ROT_STIFFNESS_RZ   = 1.0   # Nm/deg     [원래: 0.5]
ROT_DAMPING_RXRY   = 0.10  # Nm*s/deg   [원래: 0.25] => K/D = 10/s
ROT_DAMPING_RZ     = 0.10  # Nm*s/deg   [원래: 0.25] => K/D = 10/s

# ---------------------------------------------------------------------------
# [MODIFIED] Safety clamps per control loop
#
# MAX_COMMAND_STEP: 한 루프당 최대 이동량
# 원래 5.0mm / 1.0deg → 유지 (충분히 크므로 bottleneck 아님)
# 반응성이 여전히 부족하면 이 값을 먼저 확인할 것
# ---------------------------------------------------------------------------
MAX_VIRTUAL_STEP_MM  = 5.0
MAX_VIRTUAL_STEP_DEG = 2.0   # [원래: 1.0] — 회전 virtual step 허용 범위 확대
MAX_COMMAND_STEP_MM  = 5.0
MAX_COMMAND_STEP_DEG = 2.0   # [원래: 1.0] — 회전 command step 허용 범위 확대

# ---------------------------------------------------------------------------
# [MODIFIED] Teleop velocity ratio
#
# 원래: 0.5 → 변경: 0.8
# 로봇이 MoveTeleL 명령을 실행할 때의 최대 속도 비율 (0~1)
# 높일수록 실제 로봇 이동 속도 증가 → 체감 반응성 직접적으로 향상
# 단, 너무 높으면 안전 제한에 걸릴 수 있음; 0.7~0.9 범위에서 테스트 권장
# ---------------------------------------------------------------------------
TEL_VEL_RATIO = 0.8   # [원래: 0.5]
TEL_ACC_RATIO = 1.0

CONTROL_PERIOD = 0.02
MAX_DT = CONTROL_PERIOD * 2

AXIS_NAMES = ['X(tool)', 'Y(tool)', 'Z(tool)', 'Rx(tool)', 'Ry(tool)', 'Rz(tool)']
AXIS_UNITS = ['mm', 'mm', 'mm', 'deg', 'deg', 'deg']
AXIS_MODE_TO_INDICES = {
    'X': (0, 3),
    'Y': (1, 4),
    'Z': (2, 5),
    'ALL': (0, 1, 2, 3, 4, 5),
}


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
    """기존 단순 deadband (내부 헬퍼로 유지)."""
    if abs(val) < threshold:
        return 0.0
    return (val - threshold) if val > 0 else (val + threshold)


def get_enabled_axis_indices():
    mode = AXIS_TEST_MODE.upper()
    if mode not in AXIS_MODE_TO_INDICES:
        raise ValueError("AXIS_TEST_MODE must be one of 'X', 'Y', 'Z', 'ALL'")
    return AXIS_MODE_TO_INDICES[mode]


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


# ---------------------------------------------------------------------------
# [NEW] 2-Stage Hysteresis Deadband
# ---------------------------------------------------------------------------

def hysteresis_deadband(val, engage, release, is_active):
    """
    2단계 히스테리시스 deadband.

    Parameters
    ----------
    val      : 현재 bias-compensated 힘/토크 값
    engage   : 정지 → 운동 전환 임계값  (예: 2.0 N)
    release  : 운동 → 정지 전환 임계값  (예: 1.0 N)
    is_active: 해당 축의 현재 운동 상태 (True=운동중, False=정지)

    Returns
    -------
    (effective_value, new_is_active)
      effective_value : deadband 처리 후 실제 사용할 값
      new_is_active   : 갱신된 운동 상태 플래그
    """
    abs_val = abs(val)

    if not is_active:
        # 정지 상태: ENGAGE 임계값 이상이어야 운동 시작
        if abs_val >= engage:
            new_active = True
            eff = (val - engage) if val > 0 else (val + engage)
        else:
            new_active = False
            eff = 0.0
    else:
        # 운동 상태: RELEASE 임계값 미만이면 정지로 전환
        if abs_val >= release:
            new_active = True
            eff = (val - release) if val > 0 else (val + release)
        else:
            new_active = False
            eff = 0.0

    return eff, new_active


def compensate_and_hysteresis(ft_raw, bias, enabled_indices, axis_active):
    """
    bias 보정 후 2단계 히스테리시스 deadband 적용.

    Parameters
    ----------
    ft_raw        : 센서 원시값 [Fx, Fy, Fz, Tx, Ty, Tz]
    bias          : 바이어스 [6]
    enabled_indices: 활성화된 축 인덱스
    axis_active   : 각 축의 현재 운동 상태 플래그 [bool * 6]  ← in-place 수정됨

    Returns
    -------
    ft_comp   : bias 보정된 원시값 [6]
    wrench_eff: 히스테리시스 적용 후 유효 wrench [6]
    """
    ft_comp = [ft_raw[i] - bias[i] for i in range(6)]

    # (engage, release) 쌍을 축 인덱스에 매핑
    thresholds = [
        (FORCE_ENGAGE_XY,      FORCE_RELEASE_XY),      # 0: Fx
        (FORCE_ENGAGE_XY,      FORCE_RELEASE_XY),      # 1: Fy
        (FORCE_ENGAGE_Z,       FORCE_RELEASE_Z),       # 2: Fz
        (TORQUE_ENGAGE_RXRY,   TORQUE_RELEASE_RXRY),   # 3: Tx
        (TORQUE_ENGAGE_RXRY,   TORQUE_RELEASE_RXRY),   # 4: Ty
        (TORQUE_ENGAGE_RZ,     TORQUE_RELEASE_RZ),     # 5: Tz
    ]

    enabled = set(enabled_indices)
    wrench_eff = [0.0] * 6

    for i in range(6):
        if i not in enabled:
            axis_active[i] = False
            continue
        engage, release = thresholds[i]
        eff, new_active = hysteresis_deadband(ft_comp[i], engage, release, axis_active[i])
        axis_active[i] = new_active
        wrench_eff[i] = eff

    return ft_comp, wrench_eff


def update_virtual_target(virtual_pose, wrench_eff, dt, enabled_indices):
    # type: (List[float], List[float], float, tuple) -> List[float]
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

    enabled = set(enabled_indices)
    for i in range(6):
        if i in enabled:
            virtual_pose[i] += virtual_step[i]
        else:
            virtual_step[i] = 0.0

    return virtual_step


def compute_spring_damper_step(virtual_pose, command_pose, dt, enabled_indices):
    # type: (List[float], List[float], float, tuple) -> Tuple[List[float], List[float]]
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

    enabled = set(enabled_indices)
    for i in range(6):
        if i not in enabled:
            error[i] = 0.0
            command_step[i] = 0.0

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
               virtual_pose, command_pose, error, loop_count, axis_active):
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

    # [MODIFIED] axis_active 상태 출력 추가
    active_str = ''.join('A' if axis_active[i] else '_' for i in range(6))

    log.debug(
        '[Loop %4d] RAW F=[%+6.2f,%+6.2f,%+6.2f]N RAW T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[Loop %4d] COMP F=[%+6.2f,%+6.2f,%+6.2f]N COMP T=[%+6.3f,%+6.3f,%+6.3f]Nm '
        'EFF=[%+6.2f,%+6.2f,%+6.2f,%+6.3f,%+6.3f,%+6.3f] ACTIVE=%s',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
        wrench_eff[0], wrench_eff[1], wrench_eff[2],
        wrench_eff[3], wrench_eff[4], wrench_eff[5],
        active_str,
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
    try:
        enabled_indices = get_enabled_axis_indices()
    except ValueError as e:
        log.error('%s', e)
        return

    enabled_axis_names = ', '.join(AXIS_NAMES[i] for i in enabled_indices)

    log.info('=' * 70)
    log.info('6-axis virtual-point admittance controller started')
    log.info('Robot command mode: %s', 'APPLY' if APPLY_ROBOT_COMMANDS else 'DEBUG_ONLY')
    log.info('Axis test mode: %s (%s)', AXIS_TEST_MODE.upper(), enabled_axis_names)
    log.info('Model: virtual target from F/T, follow with D*x_dot = K*(x_virtual - x_command)')
    log.info('[HYSTERESIS] ENGAGE F_xy/z=%.2f/%.2f N, RELEASE F_xy/z=%.2f/%.2f N',
             FORCE_ENGAGE_XY, FORCE_ENGAGE_Z, FORCE_RELEASE_XY, FORCE_RELEASE_Z)
    log.info('[HYSTERESIS] ENGAGE T_rxry/rz=%.3f/%.3f Nm, RELEASE T_rxry/rz=%.3f/%.3f Nm',
             TORQUE_ENGAGE_RXRY, TORQUE_ENGAGE_RZ, TORQUE_RELEASE_RXRY, TORQUE_RELEASE_RZ)
    log.info('Input gain F xy/z=%.2f/%.2f mm/(N*s), T rxry/rz=%.2f/%.2f deg/(Nm*s)',
             VIRTUAL_POINT_FORCE_GAIN_XY, VIRTUAL_POINT_FORCE_GAIN_Z,
             VIRTUAL_POINT_TORQUE_GAIN_RXRY, VIRTUAL_POINT_TORQUE_GAIN_RZ)
    log.info('K trans xy/z=%.2f/%.2f N/mm, D trans xy/z=%.2f/%.2f N*s/mm  (K/D=%.1f/s)',
             STIFFNESS_XY, STIFFNESS_Z, DAMPING_XY, DAMPING_Z,
             STIFFNESS_XY / DAMPING_XY)
    log.info('K rot rxry/rz=%.3f/%.3f Nm/deg, D rot rxry/rz=%.3f/%.3f Nm*s/deg  (K/D=%.1f/s)',
             ROT_STIFFNESS_RXRY, ROT_STIFFNESS_RZ, ROT_DAMPING_RXRY, ROT_DAMPING_RZ,
             ROT_STIFFNESS_RXRY / ROT_DAMPING_RXRY)
    log.info('TEL_VEL_RATIO=%.2f', TEL_VEL_RATIO)
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
    # [NEW] 각 축의 운동 상태 플래그 (히스테리시스 상태 기억)
    axis_active  = [False, False, False, False, False, False]
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

            # [MODIFIED] 히스테리시스 deadband 적용 (axis_active in-place 갱신)
            ft_comp, wrench_eff = compensate_and_hysteresis(
                ft_raw, bias, enabled_indices, axis_active
            )

            virtual_step = update_virtual_target(virtual_pose, wrench_eff, dt, enabled_indices)
            command_step, error = compute_spring_damper_step(
                virtual_pose, command_pose, dt, enabled_indices
            )

            for i in range(6):
                command_pose[i] += command_step[i]

            log_status(
                ft_raw, ft_comp, wrench_eff, virtual_step, command_step,
                virtual_pose, command_pose, error, loop_count, axis_active
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