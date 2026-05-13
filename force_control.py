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
# [MODIFIED v2] Changes from v1 (hysteresis + smoothing):
#
#   핵심 문제: 기존 히스테리시스가 on/off 스위치처럼 동작해 움직임이 뚝뚝 끊김.
#
#   변경사항:
#   1. [Smooth Hysteresis] hysteresis_deadband 출력을 연속값으로 변경
#      - 정지 상태: |force| >= ENGAGE 가 되어야 운동 시작 (2단계 구조 유지)
#      - 운동 상태: |force| >= RELEASE 면 운동 유지 (작은 힘으로 유지)
#      - 출력값: threshold를 뺀 연속값 (0이 아닌 부드러운 시작)
#        예) engage=2N, force=2.5N → eff = 0.5N (갑자기 2.5N이 아닌 0.5N)
#      → 상태 전환 시 출력 불연속성 제거
#
#   2. [Wrench LPF] 센서 노이즈로 인한 threshold 경계 왔다갔다 방지
#      - wrench_eff에 1차 저역통과필터 적용
#      - alpha=WRENCH_LPF_ALPHA: 값이 클수록 현재값 반영↑ (반응 빠름), 작을수록 평탄
#
#   3. [Command Step LPF] MoveTeleL 명령 자체를 부드럽게
#      - command_step에 1차 저역통과필터 적용
#      - 갑작스런 step 변화로 인한 로봇 저크(jerk) 감소
#
# [MODIFIED v4] Changes from v3 (max force/torque threshold):
#
#   핵심 기능: bias 보정 후 외력이 최대 허용값을 초과하면 해당 루프의 명령을 무시.
#
#   동작 원리:
#   - bias 보정 완료된 ft_comp 기준으로 판단 (실제 외력 기준)
#   - F_xy, F_z, T_rxry, T_rz 각 그룹별로 독립 체크
#   - 어느 축이든 초과하면 → 해당 루프 wrench 전체 0 처리 + axis_active 전부 리셋
#   - WARN 로그 출력 (초과 시에만)
#   - virtual_pose anchor(v3)와 연계되어 초과 루프에서 추격 오차도 flush됨
#
#   튜닝 가이드:
#   - 너무 낮으면 정상적인 강한 조작도 무시됨 → 조작 의도 반영이 안 됨
#   - 너무 높으면 충돌/오작동 보호 효과가 없음
#   - 일반적으로 ENGAGE의 5~10배 수준이 적절 (F: 2N engage → 15~20N max)
#
# [MODIFIED v3] Changes from v2 (virtual_pose anchor + tuning):
#
#   핵심 문제: 손을 떼도 로봇이 계속 움직이며, 관절 90도 근처에서 특히 심하게 튐.
#
#   원인 분석:
#   - 힘이 가해지는 동안 virtual_pose가 command_pose보다 앞서 나감
#   - 손을 떼면 wrench=0이 되어 virtual_pose는 멈추지만,
#     이미 쌓인 error(virtual - command)를 spring-damper가 계속 추격
#   - 90도 근처 특이점에서 동일 task-space 명령에 관절이 훨씬 크게 반응
#
#   변경사항:
#   1. [Virtual Pose Anchor] 모든 축이 비활성(axis_active 전부 False)이면
#      virtual_pose를 command_pose로 리셋하여 잔류 오차(error) 제거
#      → 손을 떼는 즉시 추격 동작 중단
#
#   2. [Damping 강화] DAMPING_XY: 0.20 → 0.30 (K/D: 20 → ~13/s)
#      → spring-damper 추격 속도를 완만하게 해 튐 감소
#
#   3. [Command Step 클램프 강화] MAX_COMMAND_STEP_MM: 5.0 → 2.0 mm
#      → 특이점 근처에서 한 루프당 최대 이동량 제한
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

ROBOT_IP = '192.168.0.99'
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
# [유지] 2-Stage Hysteresis Thresholds
#
# ENGAGE  (start threshold): 정지 상태에서 움직임을 시작하려면 이 값 이상의 힘 필요
# RELEASE (sustain threshold): 한 번 움직이기 시작한 뒤 운동을 유지하기 위한 최소 힘
#
# [v2 변경] 출력 방식 변경: on/off → 연속값
#   - 기존: active이면 원래 force 값 그대로 출력 → 상태 전환 시 갑자기 큰 값 출력
#   - 변경: active이면 (force - threshold)를 출력 → 전환 직후 0에 가까운 값부터 시작
#
# 동작 원리 (구조는 동일):
#   axis_active[i] == False (정지):  |force| >= ENGAGE  => axis_active = True
#                                    출력 = force - ENGAGE  (ENGAGE 이상분만)
#   axis_active[i] == True  (운동):  |force| >= RELEASE => 운동 유지
#                                    출력 = force - RELEASE (RELEASE 이상분만)
#                                    |force| <  RELEASE => axis_active = False, 출력 = 0
#
# 튜닝 가이드:
#   - ENGAGE를 낮추면 작은 힘으로도 쉽게 시작됨 (민감도 ↑, 의도치 않은 움직임 위험 ↑)
#   - ENGAGE를 높이면 확실한 의도가 있어야만 시작됨 (안정성 ↑, 반응성 ↓)
#   - RELEASE를 ENGAGE 대비 40~60% 수준으로 유지하면 히스테리시스 효과가 자연스러움
# ---------------------------------------------------------------------------
FORCE_ENGAGE_XY   = 2.0    # N  — 정지 → 운동 전환에 필요한 힘
FORCE_RELEASE_XY  = 1.0    # N  — 운동 → 정지 전환 경계 (ENGAGE의 50%)

FORCE_ENGAGE_Z    = 2.0    # N
FORCE_RELEASE_Z   = 1.0    # N

TORQUE_ENGAGE_RXRY   = 0.20  # Nm
TORQUE_RELEASE_RXRY  = 0.10  # Nm

TORQUE_ENGAGE_RZ     = 0.20  # Nm
TORQUE_RELEASE_RZ    = 0.10  # Nm

# ---------------------------------------------------------------------------
# [NEW v4] Maximum Force/Torque Thresholds (초과 시 해당 루프 명령 무시)
#
# bias 보정 후 외력(ft_comp) 기준으로 판단.
# 어느 축이든 해당 그룹의 max를 초과하면 → wrench 전체 0 + axis_active 리셋.
#
# 튜닝 가이드:
#   - ENGAGE의 5~10배 수준 권장
#   - 실제 작업 최대 조작력 + 충분한 여유 (예: 최대 조작 5N → MAX 15~20N)
#   - 너무 낮으면 정상 조작이 무시됨, 너무 높으면 보호 효과 없음
# ---------------------------------------------------------------------------
FORCE_MAX_XY  = 20.0   # N  — Fx 또는 Fy 초과 시 무시
FORCE_MAX_Z   = 20.0   # N  — Fz 초과 시 무시
TORQUE_MAX_RXRY = 2.0  # Nm — Tx 또는 Ty 초과 시 무시
TORQUE_MAX_RZ   = 2.0  # Nm — Tz 초과 시 무시

# ---------------------------------------------------------------------------
# [NEW v2] Low-Pass Filter coefficients
#
# 1차 LPF: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
#
# WRENCH_LPF_ALPHA: wrench_eff에 적용 (threshold 이후 유효 힘값)
#   - 센서 노이즈가 threshold 경계를 왔다갔다해서 생기는 미세한 on/off 진동 제거
#   - 0.3~0.6 권장: 너무 낮으면 반응 지연, 너무 높으면 필터 효과 없음
#
# CMD_LPF_ALPHA: command_step에 적용 (MoveTeleL로 보내는 명령)
#   - 갑작스런 명령 변화로 인한 로봇 저크(jerk) 감소
#   - 0.4~0.7 권장: 너무 낮으면 로봇 반응이 느려짐
# ---------------------------------------------------------------------------
WRENCH_LPF_ALPHA = 0.4   # 0~1 (클수록 현재값 반영 ↑, 작을수록 평탄)
CMD_LPF_ALPHA    = 0.5   # 0~1

# ---------------------------------------------------------------------------
# [유지] Virtual target gain — 힘 입력이 virtual target을 얼마나 빠르게 이동시키는가
# ---------------------------------------------------------------------------
VIRTUAL_POINT_FORCE_GAIN_XY    = 8.0   # mm / (N*s)
VIRTUAL_POINT_FORCE_GAIN_Z     = 6.0   # mm / (N*s)
VIRTUAL_POINT_TORQUE_GAIN_RXRY = 10.0  # deg / (Nm*s)
VIRTUAL_POINT_TORQUE_GAIN_RZ   = 10.0  # deg / (Nm*s)

# ---------------------------------------------------------------------------
# [유지] Spring-damper follow dynamics: D * x_dot = K * error
# ---------------------------------------------------------------------------
STIFFNESS_XY  = 4.0   # N/mm
STIFFNESS_Z   = 4.0   # N/mm
DAMPING_XY    = 0.30  # N*s/mm   => K/D = ~13/s  [v3: 0.20→0.30, 추격 속도 완만하게]
DAMPING_Z     = 0.20  # N*s/mm   => K/D = 20/s

ROT_STIFFNESS_RXRY = 1.0   # Nm/deg
ROT_STIFFNESS_RZ   = 1.0   # Nm/deg
ROT_DAMPING_RXRY   = 0.10  # Nm*s/deg  => K/D = 10/s
ROT_DAMPING_RZ     = 0.10  # Nm*s/deg  => K/D = 10/s

# ---------------------------------------------------------------------------
# [유지] Safety clamps per control loop
# ---------------------------------------------------------------------------
MAX_VIRTUAL_STEP_MM  = 5.0
MAX_VIRTUAL_STEP_DEG = 2.0
MAX_COMMAND_STEP_MM  = 2.0   # [v3: 5.0→2.0, 특이점 근처 한 루프당 최대 이동량 제한]
MAX_COMMAND_STEP_DEG = 2.0

# ---------------------------------------------------------------------------
# [유지] Teleop velocity ratio
# ---------------------------------------------------------------------------
TEL_VEL_RATIO = 0.8
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
# Logging — compact columnar format
# ===================================================================

class _CompactFormatter(logging.Formatter):
    """한 눈에 읽히는 고정폭 포맷.

    [HH:MM:SS.mmm] LEVEL  message
    레벨을 5자 고정폭으로 정렬하여 DEBUG/INFO/WARNING/ERROR 모두 열이 맞음.
    """
    LEVEL_ABBREV = {
        logging.DEBUG:    'DEBUG',
        logging.INFO:     'INFO ',
        logging.WARNING:  'WARN ',
        logging.ERROR:    'ERROR',
        logging.CRITICAL: 'CRIT ',
    }

    def format(self, record):
        ts = self.formatTime(record, '%H:%M:%S')
        ms = int(record.msecs)
        lvl = self.LEVEL_ABBREV.get(record.levelno, record.levelname[:5])
        msg = record.getMessage()
        return f'[{ts}.{ms:03d}] {lvl}  {msg}'


def _build_logger():
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_CompactFormatter())
    logger = logging.getLogger('VirtualAdmittance')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.propagate = False
    return logger


log = _build_logger()


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
# [MODIFIED v2] 2-Stage Smooth Hysteresis Deadband
#
# v1과 달리 출력이 연속값 (on/off 아님):
#   - 정지→운동 전환 시: eff = force - ENGAGE   (갑자기 큰 값이 아니라 0에서 시작)
#   - 운동 유지 시:      eff = force - RELEASE  (RELEASE 이상분만 유효 출력)
#   - 운동→정지 전환 시: eff = 0.0
#
# 이렇게 하면 상태 전환 순간에 출력이 연속적으로 이어지며,
# 2단계 threshold 구조(정지: 큰 힘 필요 / 운동 유지: 작은 힘으로 충분)는 그대로 유지됨.
# ---------------------------------------------------------------------------

def hysteresis_deadband(val, engage, release, is_active):
    """
    2단계 히스테리시스 deadband (연속값 출력 버전).

    Parameters
    ----------
    val      : 현재 bias-compensated 힘/토크 값
    engage   : 정지 → 운동 전환 임계값  (예: 2.0 N)
    release  : 운동 → 정지 전환 임계값  (예: 1.0 N)
    is_active: 해당 축의 현재 운동 상태 (True=운동중, False=정지)

    Returns
    -------
    (effective_value, new_is_active)
      effective_value : deadband 처리 후 연속 출력값
                        정지→운동: val - engage  (전환 직후 0 근처에서 연속 시작)
                        운동 유지: val - release (release 이상분만)
                        정지:      0.0
      new_is_active   : 갱신된 운동 상태 플래그
    """
    abs_val = abs(val)
    sign = 1.0 if val >= 0.0 else -1.0

    if not is_active:
        # 정지 상태: ENGAGE 이상이어야 운동 시작
        if abs_val >= engage:
            new_active = True
            # 전환 직후 갑자기 큰 값 대신 (abs_val - engage) 만큼만 출력
            eff = sign * (abs_val - engage)
        else:
            new_active = False
            eff = 0.0
    else:
        # 운동 상태: RELEASE 이상이면 운동 유지
        if abs_val >= release:
            new_active = True
            # RELEASE 이상분만 출력 (작은 힘도 연속으로 반영)
            eff = sign * (abs_val - release)
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
    wrench_eff: 히스테리시스 적용 후 유효 wrench [6] (연속값)
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


# ---------------------------------------------------------------------------
# [NEW v4] Max Force/Torque Guard
# ---------------------------------------------------------------------------

def check_max_wrench(ft_comp, loop_count):
    # type: (List[float], int) -> bool
    """
    bias 보정된 외력(ft_comp)이 최대 허용값을 초과하는지 검사.

    Parameters
    ----------
    ft_comp    : bias 보정된 wrench [Fx, Fy, Fz, Tx, Ty, Tz]
    loop_count : 로그용 루프 번호

    Returns
    -------
    True  : 초과 → 해당 루프 명령 무시해야 함
    False : 정상 범위
    """
    checks = [
        (0, FORCE_MAX_XY,    'Fx'),
        (1, FORCE_MAX_XY,    'Fy'),
        (2, FORCE_MAX_Z,     'Fz'),
        (3, TORQUE_MAX_RXRY, 'Tx'),
        (4, TORQUE_MAX_RXRY, 'Ty'),
        (5, TORQUE_MAX_RZ,   'Tz'),
    ]
    for idx, limit, name in checks:
        if abs(ft_comp[idx]) > limit:
            log.warning(
                '[Loop %d] MAX WRENCH EXCEEDED: %s=%.2f (limit=%.2f) — loop skipped',
                loop_count, name, ft_comp[idx], limit
            )
            return True
    return False
# ---------------------------------------------------------------------------

def apply_lpf(current, previous, alpha):
    # type: (List[float], List[float], float) -> List[float]
    """
    1차 저역통과필터: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]

    Parameters
    ----------
    current  : 현재 입력값 리스트
    previous : 이전 필터 출력값 리스트 (in-place 갱신 아님; 호출 측에서 저장)
    alpha    : 필터 계수 (0~1). 클수록 현재값 반영 ↑

    Returns
    -------
    filtered : 필터링된 출력값 리스트
    """
    return [alpha * current[i] + (1.0 - alpha) * previous[i] for i in range(len(current))]


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


def log_status(ft_raw, ft_comp, wrench_eff, wrench_filtered, virtual_step, command_step,
               command_step_filtered, virtual_pose, command_pose, error, loop_count, axis_active,
               joint_q=None):
    if loop_count % 10 != 0:
        return

    moving_axes = []
    for i, name in enumerate(AXIS_NAMES):
        if abs(command_step_filtered[i]) > 1e-4:
            direction = '+' if command_step_filtered[i] > 0 else '-'
            moving_axes.append('{}{} {:.3f}{}'.format(
                direction, name, abs(command_step_filtered[i]), AXIS_UNITS[i]
            ))
    move_str = ', '.join(moving_axes) if moving_axes else 'stop'

    active_str = ''.join('A' if axis_active[i] else '_' for i in range(6))

    log.debug(
        '[Loop %4d] RAW F=[%+6.2f,%+6.2f,%+6.2f]N RAW T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[Loop %4d] COMP F=[%+6.2f,%+6.2f,%+6.2f]N COMP T=[%+6.3f,%+6.3f,%+6.3f]Nm '
        'EFF=[%+6.2f,%+6.2f,%+6.2f,%+6.3f,%+6.3f,%+6.3f] '
        'EFF_LPF=[%+6.2f,%+6.2f,%+6.2f,%+6.3f,%+6.3f,%+6.3f] ACTIVE=%s',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
        wrench_eff[0], wrench_eff[1], wrench_eff[2],
        wrench_eff[3], wrench_eff[4], wrench_eff[5],
        wrench_filtered[0], wrench_filtered[1], wrench_filtered[2],
        wrench_filtered[3], wrench_filtered[4], wrench_filtered[5],
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
        'cmd_step_lpf=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '
        'cmd_pose=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg axes=%s',
        loop_count,
        command_step[0], command_step[1], command_step[2],
        command_step[3], command_step[4], command_step[5],
        command_step_filtered[0], command_step_filtered[1], command_step_filtered[2],
        command_step_filtered[3], command_step_filtered[4], command_step_filtered[5],
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
    if joint_q is not None:
        q_str = ', '.join('%+8.4f' % v for v in joint_q)
        log.debug('[Loop %4d] JOINT q=[%s] deg', loop_count, q_str)


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
    log.info('6-axis virtual-point admittance controller started  [v4: max wrench guard]')
    log.info('Robot command mode: %s', 'APPLY' if APPLY_ROBOT_COMMANDS else 'DEBUG_ONLY')
    log.info('Axis test mode: %s (%s)', AXIS_TEST_MODE.upper(), enabled_axis_names)
    log.info('Model: virtual target from F/T, follow with D*x_dot = K*(x_virtual - x_command)')
    log.info('[HYSTERESIS] ENGAGE F_xy/z=%.2f/%.2f N, RELEASE F_xy/z=%.2f/%.2f N',
             FORCE_ENGAGE_XY, FORCE_ENGAGE_Z, FORCE_RELEASE_XY, FORCE_RELEASE_Z)
    log.info('[HYSTERESIS] ENGAGE T_rxry/rz=%.3f/%.3f Nm, RELEASE T_rxry/rz=%.3f/%.3f Nm',
             TORQUE_ENGAGE_RXRY, TORQUE_ENGAGE_RZ, TORQUE_RELEASE_RXRY, TORQUE_RELEASE_RZ)
    log.info('[LPF] wrench_alpha=%.2f, cmd_step_alpha=%.2f', WRENCH_LPF_ALPHA, CMD_LPF_ALPHA)
    log.info('Input gain F xy/z=%.2f/%.2f mm/(N*s), T rxry/rz=%.2f/%.2f deg/(Nm*s)',
             VIRTUAL_POINT_FORCE_GAIN_XY, VIRTUAL_POINT_FORCE_GAIN_Z,
             VIRTUAL_POINT_TORQUE_GAIN_RXRY, VIRTUAL_POINT_TORQUE_GAIN_RZ)
    log.info('K trans xy/z=%.2f/%.2f N/mm, D trans xy/z=%.2f/%.2f N*s/mm  (K/D=%.1f/%.1f /s)',
             STIFFNESS_XY, STIFFNESS_Z, DAMPING_XY, DAMPING_Z,
             STIFFNESS_XY / DAMPING_XY, STIFFNESS_Z / DAMPING_Z)
    log.info('K rot rxry/rz=%.3f/%.3f Nm/deg, D rot rxry/rz=%.3f/%.3f Nm*s/deg  (K/D=%.1f/s)',
             ROT_STIFFNESS_RXRY, ROT_STIFFNESS_RZ, ROT_DAMPING_RXRY, ROT_DAMPING_RZ,
             ROT_STIFFNESS_RXRY / ROT_DAMPING_RXRY)
    log.info('TEL_VEL_RATIO=%.2f', TEL_VEL_RATIO)
    log.info('[v3] Virtual anchor ON (손 뗌 즉시 오차 flush), MAX_CMD_STEP=%.1fmm/%.1fdeg',
             MAX_COMMAND_STEP_MM, MAX_COMMAND_STEP_DEG)
    log.info('[v4] MAX WRENCH F_xy/z=%.1f/%.1f N, T_rxry/rz=%.3f/%.3f Nm  (초과 시 루프 무시)',
             FORCE_MAX_XY, FORCE_MAX_Z, TORQUE_MAX_RXRY, TORQUE_MAX_RZ)
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

    time.sleep(2)
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
    virtual_pose  = [0.0] * 6
    command_pose  = [0.0] * 6
    axis_active   = [False] * 6

    # [NEW v2] LPF 이전 출력값 초기화
    wrench_filtered      = [0.0] * 6   # wrench_eff LPF 상태
    cmd_step_filtered    = [0.0] * 6   # command_step LPF 상태

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

            # 관절 각도 읽기 (로봇 연결 시에만)
            joint_q = None
            if APPLY_ROBOT_COMMANDS and indy is not None:
                try:
                    joint_q = indy.get_control_state()['q']
                except Exception:
                    pass

            # 히스테리시스 deadband 적용 (연속값 출력)
            ft_comp, wrench_eff = compensate_and_hysteresis(
                ft_raw, bias, enabled_indices, axis_active
            )

            # [NEW v4] Max wrench guard: 외력이 최대값 초과 시 해당 루프 명령 무시
            if check_max_wrench(ft_comp, loop_count):
                wrench_eff = [0.0] * 6
                for i in range(6):
                    axis_active[i] = False
                    virtual_pose[i] = command_pose[i]  # anchor도 함께 flush
                loop_count += 1
                elapsed = time.time() - t_start
                sleep_time = CONTROL_PERIOD - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            # [NEW v2] wrench LPF: 센서 노이즈로 인한 경계 진동 방지
            wrench_filtered = apply_lpf(wrench_eff, wrench_filtered, WRENCH_LPF_ALPHA)

            # 필터링된 wrench로 virtual target 갱신
            virtual_step = update_virtual_target(virtual_pose, wrench_filtered, dt, enabled_indices)

            # [NEW v3] Virtual Pose Anchor:
            # 모든 축이 비활성 상태(손을 뗀 상태)이면 virtual_pose를 command_pose로
            # 리셋해서 잔류 오차(virtual - command)가 사라지게 함.
            # → 손을 떼는 즉시 spring-damper 추격 동작이 멈춤.
            if not any(axis_active):
                for i in range(6):
                    virtual_pose[i] = command_pose[i]

            # spring-damper로 command step 계산
            command_step, error = compute_spring_damper_step(
                virtual_pose, command_pose, dt, enabled_indices
            )

            # [NEW v2] command step LPF: 명령 자체의 저크 감소
            cmd_step_filtered = apply_lpf(command_step, cmd_step_filtered, CMD_LPF_ALPHA)

            # 필터링된 command step으로 pose 누적
            for i in range(6):
                command_pose[i] += cmd_step_filtered[i]

            log_status(
                ft_raw, ft_comp, wrench_eff, wrench_filtered,
                virtual_step, command_step, cmd_step_filtered,
                virtual_pose, command_pose, error, loop_count, axis_active,
                joint_q=joint_q,
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