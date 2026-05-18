# ***********************************************************************
#
# Nuri 원격 조작을 위한 6축 F/T 가상 포인트 어드미턴스 컨트롤러
# (Python 3.7 호환)
#
# 제어 개념:
# 1. Bias가 보정된 힘/토크가 가상 목표 포즈(virtual target pose)를 이동시킨다.
# 2. 로봇 명령 포즈(command pose)는 2차 어드미턴스(질량-감쇠-스프링) 모델로
#    가상 목표를 추종한다:
#
#    M * x_ddot + D * x_dot + K * (x_command - x_virtual) = 0
#    → 이산화: x_dot += (dt/M) * (-D * x_dot - K * error)
#
# 3. MoveTeleL은 누적된 6D 상대 작업 포즈를 수신한다:
#    [x, y, z, Rx, Ry, Rz]
#
# 주의사항:
# - 병진 단위는 mm.
# - 회전 단위는 Indy 작업 포즈 기준 deg로 가정.
# - 실제 TCP 포즈는 마지막으로 명령한 상대 포즈로 근사된다.
#   나중에 로봇 피드백 포즈가 필요한 경우, 스프링-댐퍼 오차 계산에서
#   command_pose를 측정된 TCP 상대 포즈로 교체할 것.
#
# 로봇 속도 = (K/D) × G × 힘   (정상상태, 질량항 무시 시)
#             ↑       ↑   ↑
#             추종 감도  핸들 감도  내가 민 힘
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
# 하드웨어 설정
# ===================================================================

CAN_INTERFACE = 'slcan'         # CAN 인터페이스 종류
CAN_CHANNEL   = 'COM3'          # CAN 연결 포트
CAN_BITRATE   = 1_000_000       # CAN 통신 속도 (bps)

# AFT200-D80 기본 CAN ID (공장값 0x01)
# 힘 프레임: arbitration_id == SENSOR_CAN_ID
# 토크 프레임: arbitration_id == SENSOR_CAN_ID + 1
SENSOR_CAN_ID = 0x001           # 센서 기본 CAN ID (힘 프레임)

ROBOT_IP    = '192.168.0.99'    # 로봇 IP 주소
ROBOT_INDEX = 0                 # 로봇 인덱스 번호

# False: F/T 값, 가상 목표, 계산된 명령 방향만 출력 (로봇 미동작).
# True:  원격 조작 모드로 진입하여 로봇에 MoveTeleL 명령 전송.
APPLY_ROBOT_COMMANDS = True

# 안전한 단계별 테스트를 위한 축 고립 설정.
# 'X' -> Fx와 Tx 축만 사용
# 'Y' -> Fy와 Ty 축만 사용
# 'Z' -> Fz와 Tz 축만 사용
# 'ALL' -> 모든 힘/토크 축 사용
AXIS_TEST_MODE = 'ALL'

# ===================================================================
# 제어 파라미터
# ===================================================================

BIAS_SAMPLE_COUNT = 200         # Bias 측정에 사용할 샘플 수
BIAS_SAMPLE_DELAY = 0.005       # 샘플 수집 간격 (초)

# 입력 불감대(deadband): 이 범위 내의 값은 노이즈로 간주하여 무시.
FORCE_THRESHOLD   = 0.5         # N  (힘 불감대)
TORQUE_THRESHOLD  = 0.05        # Nm (토크 불감대)

# 조작자의 렌치(wrench)가 가상 목표를 이동시키는 속도.
# 로봇이 목표를 추종하기 전의 "핸들 감도" 역할.
VIRTUAL_POINT_FORCE_GAIN  = 2.0  # mm / (N·s)   — 손으로 민 힘 → 가상 목표 이동 속도
VIRTUAL_POINT_TORQUE_GAIN = 1.0  # deg / (Nm·s) — 토크 → 가상 목표 회전 속도
# ※ 복강경 sheath(17cm J형) 선단 증폭 효과를 감안해 기존 3.0→2.0으로 낮춤.

# -----------------------------------------------------------------------
# 2차 어드미턴스 파라미터 (질량-감쇠-스프링)
#
# M * x_ddot + D * x_dot + K * (x_cmd - x_virt) = 0
#
# [병진]
#  M_t : 가상 관성 질량 (단위: kg 환산)
#        물리적 의미: AFT200-D80 어댑터 + 플랜지 실질량 ≈ 0.3~0.5 kg
#        권장 초기값: 0.5  (응답이 느리면 낮추고, 떨림이 심하면 높임)
#  D_t : 가상 감쇠 계수
#        권장: 2*sqrt(M_t * K_t) × ζ  (ζ≈0.7~1.0, 임계제동 근방)
#        → K_t=0.5, M_t=0.5 → D_t_crit = 2*sqrt(0.25)=1.0 → D_t=0.7~1.0
#  K_t : 가상 스프링 강성
#        권장: 0.3~0.8 N/mm (복강경 환경에서는 0.5 이하 권장)
#
# [회전]
#  M_r : 가상 관성 모멘트
#        물리적 의미: AFT200-D80 + 플랜지 회전 관성 ≈ 0.005~0.02 kg·m²
#        권장 초기값: 0.01 (deg 단위이므로 스케일 조정 필요, 아래 참조)
#  D_r : 가상 회전 감쇠
#  K_r : 가상 회전 강성
#
# ※ Indy 단위계 (mm, deg) 기준이므로 SI 단위와 수치 스케일이 다름.
#   실험적으로 조정하는 것을 권장.
# -----------------------------------------------------------------------
MASS_TRANS      = 0.5           # 병진 가상 질량  [kg 환산, mm 기준]
DAMPING_TRANS   = 0.8           # 병진 감쇠 계수  [N·s/mm]
STIFFNESS_TRANS = 0.5           # 병진 스프링 강성 [N/mm]

MASS_ROT        = 0.01          # 회전 가상 관성  [kg·m² 환산, deg 기준]
DAMPING_ROT     = 0.02          # 회전 감쇠 계수  [Nm·s/deg]
STIFFNESS_ROT   = 0.05          # 회전 스프링 강성 [Nm/deg]

# 제어 루프 1회당 안전 제한값 (한 루프에서 이 값 이상 이동 불가).
MAX_VIRTUAL_STEP_MM   = 3.0     # 가상 목표 최대 병진 이동량 (mm) — 기존 5.0→3.0
MAX_VIRTUAL_STEP_DEG  = 1.0     # 가상 목표 최대 회전량 (deg)
MAX_COMMAND_STEP_MM   = 1.5     # 명령 포즈 최대 병진 이동량 (mm) — 기존 3.0→1.5 (수술 환경)
MAX_COMMAND_STEP_DEG  = 0.5     # 명령 포즈 최대 회전량 (deg)

# workspace 절대 한계 — virtual_pose가 이 범위를 벗어나면 더 이상 누적하지 않음.
MAX_WORKSPACE_MM  = 200.0       # 병진 최대 누적 변위 (mm)
MAX_WORKSPACE_DEG = 45.0        # 회전 최대 누적 변위 (deg)

TEL_VEL_RATIO = 0.5             # 텔레오퍼레이션 속도 비율 (0~1)
TEL_ACC_RATIO = 0.5             # 텔레오퍼레이션 가속도 비율 (0~1)

CONTROL_PERIOD = 0.02           # 제어 루프 주기 (초, 50 Hz)
MAX_DT = CONTROL_PERIOD * 2     # dt 스파이크 허용 최대값; 초과 시 명목 주기로 대체

AXIS_NAMES = ['X(tool)', 'Y(tool)', 'Z(tool)', 'Rx(tool)', 'Ry(tool)', 'Rz(tool)']
AXIS_UNITS = ['mm', 'mm', 'mm', 'deg', 'deg', 'deg']
AXIS_MODE_TO_INDICES = {
    'X':   (0, 3),
    'Y':   (1, 4),
    'Z':   (2, 5),
    'ALL': (0, 1, 2, 3, 4, 5),
}

# ===================================================================
# 로깅 설정
# ===================================================================

logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger('VirtualAdmittance')

# ===================================================================
# F/T 센서 리더 클래스
# ===================================================================

class FTSensorReader(object):
    def __init__(self, interface, channel, bitrate):
        self._interface = interface
        self._channel   = channel
        self._bitrate   = bitrate
        self._lock      = threading.Lock()
        self._Fx = self._Fy = self._Fz = 0.0
        self._Tx = self._Ty = self._Tz = 0.0
        self._running   = False
        self._thread    = None
        self._bus       = None
        self._new_data  = threading.Event()

    def start(self):
        """CAN 버스를 초기화하고 수신 스레드를 시작한다."""
        log.info('CAN 버스 초기화: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._send_start_command()
        self._running = True
        self._thread  = threading.Thread(
            target=self._recv_loop, name='FTReader', daemon=True
        )
        self._thread.start()
        log.info('F/T 센서 수신 스레드 시작됨')

    def stop(self):
        """수신 스레드를 종료하고 CAN 버스 연결을 닫는다."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._bus is not None:
            self._bus.shutdown()
        log.info('F/T 센서 수신 스레드 종료됨')

    def _send_start_command(self):
        """
        매뉴얼 기준 연속 전송 시작 명령:
          arbitration_id = 0x102
          data           = [SENSOR_CAN_ID, 0x03, 0x01]
                            ^               ^     ^
                            현재 센서 ID    모드  온도보상 포함
        """
        cmd = can.Message(
            arbitration_id=0x102,
            data=[SENSOR_CAN_ID, 0x03, 0x01],
            is_extended_id=False,
        )
        try:
            self._bus.send(cmd)
            log.debug('센서 연속 전송 시작 명령 전송 완료 (arb_id=0x102, data=%s)',
                      [SENSOR_CAN_ID, 0x03, 0x01])
        except can.CanError as e:
            log.warning('센서 시작 명령 실패 (이미 스트리밍 중이면 무시 가능): %s', e)

    def _recv_loop(self):
        """
        백그라운드에서 CAN 메시지를 수신하고 힘/토크 값을 파싱하는 루프.

        힘 프레임 : arbitration_id == SENSOR_CAN_ID     (0x001)
        토크 프레임: arbitration_id == SENSOR_CAN_ID + 1 (0x002)
        ※ 실제 센서 출력 ID가 다를 경우 SENSOR_CAN_ID 값을 조정하세요.
        """
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None:
                    log.warning('[CAN] 수신 타임아웃 - 센서 연결을 확인하세요')
                    continue

                d = msg.data
                if msg.arbitration_id == SENSOR_CAN_ID:
                    with self._lock:
                        self._Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
                        self._Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
                        self._Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
                    self._new_data.set()
                elif msg.arbitration_id == SENSOR_CAN_ID + 1:
                    with self._lock:
                        self._Tx = (d[0] * 256 + d[1]) / 500.0 - 50.0
                        self._Ty = (d[2] * 256 + d[3]) / 500.0 - 50.0
                        self._Tz = (d[4] * 256 + d[5]) / 500.0 - 50.0
                    self._new_data.set()

            except Exception as e:
                if self._running:
                    log.error('[CAN] 수신 오류: %s', e)

    def get_ft(self):
        """현재 힘/토크 값을 스레드 안전하게 반환한다. [Fx, Fy, Fz, Tx, Ty, Tz]"""
        with self._lock:
            return [self._Fx, self._Fy, self._Fz,
                    self._Tx, self._Ty, self._Tz]

    def wait_for_data(self, timeout=5.0):
        """첫 번째 센서 데이터가 도착할 때까지 대기한다. (타임아웃: 초 단위)"""
        return self._new_data.wait(timeout=timeout)


# ===================================================================
# 제어 보조 함수
# ===================================================================

def clip(val, limit):
    """값을 [-limit, +limit] 범위로 클리핑한다."""
    return max(-limit, min(limit, val))


def deadband(val, threshold):
    """불감대 처리: |val| < threshold이면 0 반환."""
    if abs(val) < threshold:
        return 0.0
    return (val - threshold) if val > 0 else (val + threshold)


def get_enabled_axis_indices():
    """AXIS_TEST_MODE 설정에 따라 활성화할 축 인덱스 tuple을 반환한다."""
    mode = AXIS_TEST_MODE.upper()
    if mode not in AXIS_MODE_TO_INDICES:
        raise ValueError("AXIS_TEST_MODE는 'X', 'Y', 'Z', 'ALL' 중 하나여야 합니다")
    return AXIS_MODE_TO_INDICES[mode]


def measure_bias(sensor, n_samples=BIAS_SAMPLE_COUNT, delay=BIAS_SAMPLE_DELAY):
    """
    로봇과 센서를 정지 상태로 유지하면서 F/T Bias를 측정한다.
    n_samples개의 샘플 평균을 Bias로 반환한다.
    """
    log.info('Bias 측정 중: %d개 샘플 수집. 로봇과 센서를 정지시키세요.', n_samples)
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


def compensate_and_deadband(ft_raw, bias, enabled_indices):
    """
    원시 F/T 값에서 Bias를 빼고 불감대를 적용한다.
    비활성화된 축은 유효 렌치값을 0으로 강제한다.
    """
    ft_comp = [ft_raw[i] - bias[i] for i in range(6)]
    wrench_eff = [
        deadband(ft_comp[0], FORCE_THRESHOLD),
        deadband(ft_comp[1], FORCE_THRESHOLD),
        deadband(ft_comp[2], FORCE_THRESHOLD),
        deadband(ft_comp[3], TORQUE_THRESHOLD),
        deadband(ft_comp[4], TORQUE_THRESHOLD),
        deadband(ft_comp[5], TORQUE_THRESHOLD),
    ]
    enabled = set(enabled_indices)
    for i in range(6):
        if i not in enabled:
            wrench_eff[i] = 0.0
    return ft_comp, wrench_eff


def update_virtual_target(virtual_pose, wrench_eff, dt, enabled_indices):
    """
    유효 렌치값으로 가상 목표 포즈를 한 Step 업데이트한다.
    각 축의 이동량은 안전 한계값으로 클리핑된다.
    workspace 절대 한계(MAX_WORKSPACE_*)를 초과하면 해당 축 누적을 막는다.
    """
    virtual_step = [
        wrench_eff[0] * VIRTUAL_POINT_FORCE_GAIN  * dt,
        wrench_eff[1] * VIRTUAL_POINT_FORCE_GAIN  * dt,
        wrench_eff[2] * VIRTUAL_POINT_FORCE_GAIN  * dt,
        wrench_eff[3] * VIRTUAL_POINT_TORQUE_GAIN * dt,
        wrench_eff[4] * VIRTUAL_POINT_TORQUE_GAIN * dt,
        wrench_eff[5] * VIRTUAL_POINT_TORQUE_GAIN * dt,
    ]

    for i in range(3):
        virtual_step[i] = clip(virtual_step[i], MAX_VIRTUAL_STEP_MM)
    for i in range(3, 6):
        virtual_step[i] = clip(virtual_step[i], MAX_VIRTUAL_STEP_DEG)

    enabled = set(enabled_indices)
    for i in range(6):
        if i not in enabled:
            virtual_step[i] = 0.0
            continue
        new_val = virtual_pose[i] + virtual_step[i]
        limit   = MAX_WORKSPACE_MM if i < 3 else MAX_WORKSPACE_DEG
        if abs(new_val) > limit:
            virtual_step[i] = 0.0  # workspace 한계 초과 시 누적 중단
        else:
            virtual_pose[i] = new_val

    return virtual_step


def compute_admittance_step(virtual_pose, command_pose, vel_pose, dt, enabled_indices):
    # type: (List[float], List[float], List[float], float, tuple) -> Tuple[List[float], List[float]]
    """
    2차 어드미턴스 모델(질량-감쇠-스프링)로 명령 포즈의 한 Step을 계산한다.

    운동방정식:  M * x_ddot + D * x_dot + K * (x_cmd - x_virt) = 0
    이산화:
        error     = x_virt - x_cmd              (스프링 복원력 방향)
        x_ddot    = (1/M) * (K*error - D*x_dot)
        x_dot_new = x_dot + x_ddot * dt
        x_cmd_new = x_cmd + x_dot_new * dt

    vel_pose: 명령 포즈의 현재 속도 상태 (6D), 루프 밖에서 유지해야 함.
    반환: command_step (6D 이동량), error (6D 오차)
    """
    mass_arr   = [MASS_TRANS]   * 3 + [MASS_ROT]   * 3
    damp_arr   = [DAMPING_TRANS]* 3 + [DAMPING_ROT]* 3
    stiff_arr  = [STIFFNESS_TRANS]*3 + [STIFFNESS_ROT]*3

    error        = [virtual_pose[i] - command_pose[i] for i in range(6)]
    command_step = [0.0] * 6
    enabled      = set(enabled_indices)

    for i in range(6):
        if i not in enabled:
            vel_pose[i]    = 0.0
            error[i]       = 0.0
            command_step[i]= 0.0
            continue
        accel          = (stiff_arr[i] * error[i] - damp_arr[i] * vel_pose[i]) / mass_arr[i]
        vel_pose[i]   += accel * dt
        step           = vel_pose[i] * dt
        limit          = MAX_COMMAND_STEP_MM if i < 3 else MAX_COMMAND_STEP_DEG
        command_step[i]= clip(step, limit)
        # 클리핑으로 실제 이동량이 줄어들면 속도도 일관성 유지
        if abs(step) > 1e-9:
            vel_pose[i] *= abs(command_step[i]) / abs(step)

    return command_step, error


def check_robot_connection(indy):
    """로봇의 동작 상태(op_state)와 시뮬레이션 모드를 확인한다."""
    try:
        robot_data = indy.get_robot_data()
        op_state   = robot_data.get('op_state', -1)
        sim_mode   = robot_data.get('sim_mode', False)
        log.info('로봇 상태: op_state=%d, sim_mode=%s', op_state, sim_mode)
        abnormal = {0, 2, 3, 8, 15}
        if op_state in abnormal:
            log.error('로봇이 비정상 상태입니다: op_state=%d', op_state)
            return False
        if sim_mode:
            log.warning('시뮬레이션 모드가 활성화되어 있습니다. 실제 로봇이 움직이지 않을 수 있습니다.')
        return True
    except Exception as e:
        log.error('로봇 연결 확인 실패: %s', e)
        return False


def log_status(ft_raw, ft_comp, wrench_eff, virtual_step, command_step,
               virtual_pose, command_pose, error, loop_count):
    """10 루프마다 제어 상태를 디버그 로그로 출력한다."""
    if loop_count % 10 != 0:
        return
    moving_axes = []
    for i, name in enumerate(AXIS_NAMES):
        if abs(command_step[i]) > 1e-4:
            direction = '+' if command_step[i] > 0 else '-'
            moving_axes.append('{}{} {:.3f}{}'.format(
                direction, name, abs(command_step[i]), AXIS_UNITS[i]))
    move_str = ', '.join(moving_axes) if moving_axes else 'stop'

    log.debug(
        '[루프 %4d] 원시 F=[%+6.2f,%+6.2f,%+6.2f]N 원시 T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[루프 %4d] 보정 F=[%+6.2f,%+6.2f,%+6.2f]N 유효렌치=[%+6.2f,%+6.2f,%+6.2f,%+6.3f,%+6.3f,%+6.3f]',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        wrench_eff[0], wrench_eff[1], wrench_eff[2],
        wrench_eff[3], wrench_eff[4], wrench_eff[5],
    )
    log.debug(
        '[루프 %4d] 가상Step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '        '오차=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg',
        loop_count,
        virtual_step[0], virtual_step[1], virtual_step[2],
        virtual_step[3], virtual_step[4], virtual_step[5],
        error[0], error[1], error[2], error[3], error[4], error[5],
    )
    log.debug(
        '[루프 %4d] 명령Step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '        '명령포즈=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg 이동축=%s',
        loop_count,
        command_step[0], command_step[1], command_step[2],
        command_step[3], command_step[4], command_step[5],
        command_pose[0], command_pose[1], command_pose[2],
        command_pose[3], command_pose[4], command_pose[5],
        move_str,
    )
    log.debug(
        '[루프 %4d] 가상포즈=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg',
        loop_count,
        virtual_pose[0], virtual_pose[1], virtual_pose[2],
        virtual_pose[3], virtual_pose[4], virtual_pose[5],
    )


# ===================================================================
# 메인 제어 루프
# ===================================================================

def main():
    try:
        enabled_indices = get_enabled_axis_indices()
    except ValueError as e:
        log.error('%s', e)
        return

    enabled_axis_names = ', '.join(AXIS_NAMES[i] for i in enabled_indices)

    log.info('=' * 70)
    log.info('6축 가상 포인트 어드미턴스 컨트롤러 시작')
    log.info('로봇 명령 모드: %s', '실제 제어(APPLY)' if APPLY_ROBOT_COMMANDS else '디버그 전용(DEBUG_ONLY)')
    log.info('축 테스트 모드: %s (%s)', AXIS_TEST_MODE.upper(), enabled_axis_names)
    log.info('제어 모델: M*x_ddot + D*x_dot + K*(x_cmd - x_virt) = 0  (2차 어드미턴스)')
    log.info('병진 M=%.3f D=%.3f K=%.3f', MASS_TRANS, DAMPING_TRANS, STIFFNESS_TRANS)
    log.info('회전 M=%.4f D=%.4f K=%.4f', MASS_ROT,   DAMPING_ROT,   STIFFNESS_ROT)
    log.info('입력 이득 F=%.2f mm/(N*s), T=%.2f deg/(Nm*s)',
             VIRTUAL_POINT_FORCE_GAIN, VIRTUAL_POINT_TORQUE_GAIN)
    log.info('=' * 70)

    sensor = FTSensorReader(CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE)
    try:
        sensor.start()
    except Exception as e:
        log.error('F/T 센서 초기화 실패: %s', e)
        return

    log.info('첫 번째 F/T 샘플을 기다리는 중...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T 센서 데이터 타임아웃')
        sensor.stop()
        return
    log.info('F/T 센서 데이터 수신 확인됨')

    indy = None
    if APPLY_ROBOT_COMMANDS:
        log.info('로봇 연결 중: IP=%s, index=%d', ROBOT_IP, ROBOT_INDEX)
        try:
            indy = IndyDCP3(robot_ip=ROBOT_IP, index=ROBOT_INDEX)
        except Exception as e:
            log.error('로봇 연결 실패: %s', e)
            sensor.stop()
            return
        if not check_robot_connection(indy):
            sensor.stop()
            return
    else:
        log.warning('DEBUG_ONLY 모드: 로봇 연결, 텔레오퍼레이션, MoveTeleL 명령이 모두 비활성화됩니다.')

    input('\n로봇과 F/T 센서를 정지 상태로 유지한 뒤, Enter를 눌러 Bias를 측정하세요... ')
    bias = measure_bias(sensor)

    if APPLY_ROBOT_COMMANDS:
        log.info('텔레오퍼레이션 모드 시작')
        try:
            indy.start_teleop(2)
            time.sleep(0.5)
        except Exception as e:
            log.error('start_teleop 실패: %s', e)
            sensor.stop()
            return

        teleop_state = indy.get_teleop_state()
        log.info('텔레오퍼레이션 상태: %s', teleop_state)

        robot_data = indy.get_robot_data()
        op_state   = robot_data.get('op_state', -1)
        if op_state != 17:
            log.error('텔레오퍼레이션 전환 실패: op_state=%d, 기대값 17(TELE_OP)', op_state)
            indy.stop_teleop()
            sensor.stop()
            return

    loop_count   = 0
    virtual_pose = [0.0] * 6   # 가상 목표 포즈 (누적, workspace 한계 내)
    command_pose = [0.0] * 6   # 명령 포즈 (누적)
    vel_pose     = [0.0] * 6   # 명령 포즈 속도 상태 (2차 어드미턴스용)
    prev_time    = time.time()

    log.info('제어 루프 시작. 종료하려면 Ctrl+C를 누르세요.')

    try:
        while True:
            t_start  = time.time()
            dt       = t_start - prev_time
            prev_time = t_start

            if dt > MAX_DT:
                log.debug('[루프 %d] dt 스파이크 %.1fms → %.0fms로 대체',
                          loop_count, dt * 1000, CONTROL_PERIOD * 1000)
                dt = CONTROL_PERIOD

            ft_raw = sensor.get_ft()
            ft_comp, wrench_eff = compensate_and_deadband(ft_raw, bias, enabled_indices)

            virtual_step = update_virtual_target(
                virtual_pose, wrench_eff, dt, enabled_indices
            )
            command_step, error = compute_admittance_step(
                virtual_pose, command_pose, vel_pose, dt, enabled_indices
            )

            for i in range(6):
                command_pose[i] += command_step[i]

            log_status(ft_raw, ft_comp, wrench_eff, virtual_step, command_step,
                       virtual_pose, command_pose, error, loop_count)

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
                    log.error('MoveTeleL(TCP) 오류: %s', e)
                    if not check_robot_connection(indy):
                        log.error('로봇 연결 비정상. 제어 루프를 종료합니다.')
                        break

            loop_count += 1

            elapsed    = time.time() - t_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning('[루프 %d] 제어 주기 초과: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, CONTROL_PERIOD * 1000)

    except KeyboardInterrupt:
        log.info('사용자에 의해 중단됨')
    except Exception as e:
        log.error('제어 루프 예외 발생: %s', e, exc_info=True)
    finally:
        if APPLY_ROBOT_COMMANDS and indy is not None:
            log.info('텔레오퍼레이션 종료 중...')
            try:
                indy.stop_teleop()
                time.sleep(0.3)
                log.info('텔레오퍼레이션 종료됨')
            except Exception as e:
                log.error('stop_teleop 오류: %s', e)
        else:
            log.info('DEBUG_ONLY 모드: 종료할 텔레오퍼레이션 세션 없음.')

        sensor.stop()
        log.info('시스템 종료. 총 루프 수: %d', loop_count)
        log.info('최종 가상 포즈: X=%.2f Y=%.2f Z=%.2f mm, Rx=%.2f Ry=%.2f Rz=%.2f deg',
                 virtual_pose[0], virtual_pose[1], virtual_pose[2],
                 virtual_pose[3], virtual_pose[4], virtual_pose[5])
        log.info('최종 명령 포즈: X=%.2f Y=%.2f Z=%.2f mm, Rx=%.2f Ry=%.2f Rz=%.2f deg',
                 command_pose[0], command_pose[1], command_pose[2],
                 command_pose[3], command_pose[4], command_pose[5])


if __name__ == '__main__':
    main()
