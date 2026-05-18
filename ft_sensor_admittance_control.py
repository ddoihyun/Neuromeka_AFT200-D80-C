# ***********************************************************************
#
# Indy 원격 조작을 위한 6축 F/T 가상 포인트 어드미턴스 컨트롤러
#
# 제어 개념:
#   1. Bias가 보정된 힘/토크가 가상 목표 포즈(virtual target pose)를 이동시킨다.
#   2. 로봇 명령 포즈(command pose)는 질량 항이 없는 스프링-댐퍼 관계를 통해
#      가상 목표를 추종한다:
#
#          D * x_dot = K * (x_virtual - x_command)
#
#   3. MoveTeleL은 누적된 6D 상대 작업 포즈를 수신한다:
#      [x, y, z, Rx, Ry, Rz]
#
# 주의사항:
#   - 병진 단위는 mm.
#   - 회전 단위는 Indy 작업 포즈 기준 deg로 가정.
#   - 실제 TCP 포즈는 마지막으로 명령한 상대 포즈로 근사된다.
#     나중에 로봇 피드백 포즈가 필요한 경우, 스프링-댐퍼 오차 계산에서
#     command_pose를 측정된 TCP 상대 포즈로 교체할 것.
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
# 하드웨어 설정
# ===================================================================

CAN_INTERFACE = 'slcan'       # CAN 인터페이스 종류
CAN_CHANNEL = 'COM3'          # CAN 연결 포트
CAN_BITRATE = 1_000_000       # CAN 통신 속도 (bps)

CAN_ID_FORCE = 0x001          # 힘 데이터 CAN 메시지 ID
CAN_ID_TORQUE = 0x002         # 토크 데이터 CAN 메시지 ID

ROBOT_IP = '192.168.0.99'     # 로봇 IP 주소
ROBOT_INDEX = 0               # 로봇 인덱스 번호

# 이 값 하나로 디버깅 모드와 실제 제어 모드를 전환.
# False: F/T 값, 가상 목표, 계산된 명령 방향만 출력 (로봇 미동작).
# True:  원격 조작 모드로 진입하여 로봇에 MoveTeleL 명령 전송.
APPLY_ROBOT_COMMANDS = True

# 안전한 단계별 테스트를 위한 축 고립 설정.
# 선택 가능한 값:
#   'X'   -> Fx와 Tx 축만 사용
#   'Y'   -> Fy와 Ty 축만 사용
#   'Z'   -> Fz와 Tz 축만 사용
#   'ALL' -> 모든 힘/토크 축 사용
AXIS_TEST_MODE = 'ALL'


# ===================================================================
# 제어 파라미터
# ===================================================================

BIAS_SAMPLE_COUNT = 200       # Bias 측정에 사용할 샘플 수
BIAS_SAMPLE_DELAY = 0.005     # 샘플 수집 간격 (초)

# 입력 불감대(deadband): 이 범위 내의 값은 노이즈로 간주하여 무시.
FORCE_THRESHOLD = 1.0        # N  (힘 불감대)
TORQUE_THRESHOLD = 0.1     # Nm (회전 불감대)

# 조작자의 렌치(wrench)가 가상 목표를 이동시키는 속도.
# 로봇이 목표를 추종하기 전의 "핸들 감도" 역할.
VIRTUAL_POINT_FORCE_GAIN = 10.0        # mm / (N*s)  (힘 → 가상 목표 이동 이득)
VIRTUAL_POINT_TORQUE_GAIN = 3.0      # deg / (Nm*s) (토크 → 가상 목표 회전 이득)

# 스프링-댐퍼 추종 동역학: D * x_dot = K * error
# K/D 비율이 클수록 가상 목표를 빠르게 추종함.
STIFFNESS = 8.0            # N/mm  (병진 강성)
DAMPING = 0.1             # N*s/mm (병진 감쇠)

ROT_STIFFNESS = 0.10      # Nm/deg  (회전 강성)
ROT_DAMPING = 0.05        # Nm*s/deg (회전 감쇠)

# 제어 루프 1회당 안전 제한값 (한 루프에서 이 값 이상 이동 불가).
MAX_VIRTUAL_STEP_MM = 10.0    # 가상 목표 최대 병진 이동량 (mm)
MAX_VIRTUAL_STEP_DEG = 3.0   # 가상 목표 최대 회전량 (deg)
MAX_COMMAND_STEP_MM = 10.0    # 명령 포즈 최대 병진 이동량 (mm)
MAX_COMMAND_STEP_DEG = 3.0  # 명령 포즈 최대 회전량 (deg)

TEL_VEL_RATIO = 0.8          # 텔레오퍼레이션 속도 비율 (0~1)
TEL_ACC_RATIO = 1.0          # 텔레오퍼레이션 가속도 비율 (0~1)

CONTROL_PERIOD = 0.02        # 제어 루프 주기 (초, 50 Hz)
MAX_DT = CONTROL_PERIOD * 2  # dt 스파이크 허용 최대값; 초과 시 명목 주기로 대체

AXIS_NAMES = ['X(tool)', 'Y(tool)', 'Z(tool)', 'Rx(tool)', 'Ry(tool)', 'Rz(tool)']
AXIS_UNITS = ['mm', 'mm', 'mm', 'deg', 'deg', 'deg']
# 축 테스트 모드별 활성화 인덱스 매핑 (병진 + 해당 회전 축 쌍)
AXIS_MODE_TO_INDICES = {
    'X': (0, 3),
    'Y': (1, 4),
    'Z': (2, 5),
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
        self._interface = interface      # CAN 인터페이스 이름
        self._channel = channel          # CAN 채널(포트)
        self._bitrate = bitrate          # CAN 통신 속도
        self._lock = threading.Lock()    # 센서 데이터 스레드 안전 접근을 위한 뮤텍스
        # 힘/토크 초기값 (Bias 보정 전 원시값)
        self._Fx = self._Fy = self._Fz = 0.0
        self._Tx = self._Ty = self._Tz = 0.0
        self._running = False            # 수신 스레드 동작 플래그
        self._thread = None              # 수신 백그라운드 스레드
        self._bus = None                 # CAN 버스 객체
        self._new_data = threading.Event()  # 새 데이터 수신 이벤트

    def start(self):
        """CAN 버스를 초기화하고 수신 스레드를 시작한다."""
        log.info('CAN 버스 초기화: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._send_start_command()       # 센서에 스트리밍 시작 명령 전송
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, name='FTReader', daemon=True)
        self._thread.start()
        log.info('F/T 센서 수신 스레드 시작됨')

    def stop(self):
        """수신 스레드를 종료하고 CAN 버스 연결을 닫는다."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)   # 스레드 종료 대기 (최대 3초)
        if self._bus is not None:
            self._bus.shutdown()
        log.info('F/T 센서 수신 스레드 종료됨')

    def _send_start_command(self):
        """센서에 데이터 스트리밍 시작 명령을 전송한다."""
        data = [0x04, 0x02, 0x06, 0x01, 0x03, 0x01]
        cmd = can.Message(arbitration_id=0x000, data=data, is_extended_id=False)
        try:
            self._bus.send(cmd)
            log.debug('센서 시작 명령 전송 완료')
        except can.CanError as e:
            # 이미 스트리밍 중인 경우 무시 가능
            log.warning('센서 시작 명령 실패 (이미 스트리밍 중이면 무시 가능): %s', e)

    def _recv_loop(self):
        """백그라운드에서 CAN 메시지를 수신하고 힘/토크 값을 파싱하는 루프."""
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)   # 1초 타임아웃으로 메시지 대기
                if msg is None:
                    log.warning('[CAN] 수신 타임아웃 - 센서 연결을 확인하세요')
                    continue

                if msg.arbitration_id == CAN_ID_FORCE:
                    # 힘 메시지 파싱: 2바이트 빅엔디안 → 스케일링 및 오프셋 적용
                    d = msg.data
                    with self._lock:
                        self._Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
                        self._Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
                        self._Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
                    self._new_data.set()   # 새 데이터 도착 이벤트 발생
                elif msg.arbitration_id == CAN_ID_TORQUE:
                    # 토크 메시지 파싱: 2바이트 빅엔디안 → 스케일링 및 오프셋 적용
                    d = msg.data
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
    """불감대 처리: |val| < threshold이면 0 반환, 아니면 threshold만큼 차감한 값 반환."""
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
    # indy.stop_motion(StopCategory.CAT2)
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

    반환:
        ft_comp     - Bias 보정된 값 (불감대 미적용)
        wrench_eff  - 실제 제어에 사용할 유효 렌치값 (불감대 + 축 마스크 적용)
    """
    # Bias 보정
    ft_comp = [ft_raw[i] - bias[i] for i in range(6)]
    # 각 축별 불감대 적용
    wrench_eff = [
        deadband(ft_comp[0], FORCE_THRESHOLD),
        deadband(ft_comp[1], FORCE_THRESHOLD),
        deadband(ft_comp[2], FORCE_THRESHOLD),
        deadband(ft_comp[3], TORQUE_THRESHOLD),
        deadband(ft_comp[4], TORQUE_THRESHOLD),
        deadband(ft_comp[5], TORQUE_THRESHOLD),
    ]
    # 비활성화 축은 0으로 마스킹
    enabled = set(enabled_indices)
    for i in range(6):
        if i not in enabled:
            wrench_eff[i] = 0.0
    return ft_comp, wrench_eff


def update_virtual_target(virtual_pose, wrench_eff, dt, enabled_indices):
    # type: (List[float], List[float], float) -> List[float]
    """
    유효 렌치값으로 가상 목표 포즈를 한 Step 업데이트한다.
    각 축의 이동량은 안전 한계값으로 클리핑된다.

    반환:
        virtual_step - 이번 루프에서 가상 목표가 이동한 6D 벡터
    """
    # 렌치 × 이득 × dt = 이번 루프 가상 목표 이동량
    virtual_step = [
        wrench_eff[0] * VIRTUAL_POINT_FORCE_GAIN * dt,
        wrench_eff[1] * VIRTUAL_POINT_FORCE_GAIN * dt,
        wrench_eff[2] * VIRTUAL_POINT_FORCE_GAIN * dt,
        wrench_eff[3] * VIRTUAL_POINT_TORQUE_GAIN * dt,
        wrench_eff[4] * VIRTUAL_POINT_TORQUE_GAIN * dt,
        wrench_eff[5] * VIRTUAL_POINT_TORQUE_GAIN * dt,
    ]

    # 병진(0~2) 및 회전(3~5) 이동량을 안전 한계로 클리핑
    for i in range(3):
        virtual_step[i] = clip(virtual_step[i], MAX_VIRTUAL_STEP_MM)
    for i in range(3, 6):
        virtual_step[i] = clip(virtual_step[i], MAX_VIRTUAL_STEP_DEG)

    # 활성화된 축만 가상 포즈에 누적; 비활성 축의 Step은 0으로 리셋
    enabled = set(enabled_indices)
    for i in range(6):
        if i in enabled:
            virtual_pose[i] += virtual_step[i]
        else:
            virtual_step[i] = 0.0

    return virtual_step


def compute_spring_damper_step(virtual_pose, command_pose, dt, enabled_indices):
    # type: (List[float], List[float], float) -> Tuple[List[float], List[float]]
    """
    스프링-댐퍼 모델로 명령 포즈의 한 Step 이동량을 계산한다.
    D * x_dot = K * (virtual - command) 식에서 x_dot = (K/D) * error 적용.

    반환:
        command_step - 이번 루프에서 명령 포즈가 이동해야 할 6D 벡터
        error        - 가상 목표와 명령 포즈 간의 6D 오차 벡터
    """
    # 가상 목표와 현재 명령 포즈의 오차 계산
    error = [virtual_pose[i] - command_pose[i] for i in range(6)]

    # 각 축의 K/D 비율 (스프링-댐퍼 추종 속도 결정)
    gains = [
        STIFFNESS / DAMPING, STIFFNESS / DAMPING, STIFFNESS / DAMPING,
        ROT_STIFFNESS / ROT_DAMPING, ROT_STIFFNESS / ROT_DAMPING, ROT_STIFFNESS / ROT_DAMPING,
    ]
    # 명령 Step = (K/D) * error * dt
    command_step = [gains[i] * error[i] * dt for i in range(6)]

    # 병진(0~2) 및 회전(3~5) 명령 Step을 안전 한계로 클리핑
    for i in range(3):
        command_step[i] = clip(command_step[i], MAX_COMMAND_STEP_MM)
    for i in range(3, 6):
        command_step[i] = clip(command_step[i], MAX_COMMAND_STEP_DEG)

    # 비활성화 축의 오차와 Step을 0으로 마스킹
    enabled = set(enabled_indices)
    for i in range(6):
        if i not in enabled:
            error[i] = 0.0
            command_step[i] = 0.0

    return command_step, error


def check_robot_connection(indy):
    """
    로봇의 동작 상태(op_state)와 시뮬레이션 모드를 확인한다.
    비정상 상태이면 False를 반환한다.
    """
    try:
        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        sim_mode = robot_data.get('sim_mode', False)
        log.info('로봇 상태: op_state=%d, sim_mode=%s', op_state, sim_mode)

        # 비정상으로 간주하는 op_state 코드 집합
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
    """
    10 루프마다 제어 상태를 디버그 로그로 출력한다.
    이동 중인 축 목록과 방향, 크기를 함께 표시한다.
    """
    # 10 루프에 한 번만 출력 (로그 과부하 방지)
    if loop_count % 10 != 0:
        return

    # 유의미하게 이동 중인 축 목록 구성 (임계값 1e-4 이상)
    moving_axes = []
    for i, name in enumerate(AXIS_NAMES):
        if abs(command_step[i]) > 1e-4:
            direction = '+' if command_step[i] > 0 else '-'
            moving_axes.append('{}{} {:.3f}{}'.format(
                direction, name, abs(command_step[i]), AXIS_UNITS[i]
            ))
    move_str = ', '.join(moving_axes) if moving_axes else 'stop'

    log.debug(
        '[루프 %4d] 원시 F=[%+6.2f,%+6.2f,%+6.2f]N 원시 T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[루프 %4d] 보정 F=[%+6.2f,%+6.2f,%+6.2f]N 보정 T=[%+6.3f,%+6.3f,%+6.3f]Nm '
        '유효렌치=[%+6.2f,%+6.2f,%+6.2f,%+6.3f,%+6.3f,%+6.3f]',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
        wrench_eff[0], wrench_eff[1], wrench_eff[2],
        wrench_eff[3], wrench_eff[4], wrench_eff[5],
    )
    log.debug(
        '[루프 %4d] 가상Step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '
        '오차=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg',
        loop_count,
        virtual_step[0], virtual_step[1], virtual_step[2],
        virtual_step[3], virtual_step[4], virtual_step[5],
        error[0], error[1], error[2], error[3], error[4], error[5],
    )
    log.debug(
        '[루프 %4d] 명령Step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '
        '명령포즈=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg 이동축=%s',
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
    # 설정된 축 테스트 모드로부터 활성화할 인덱스 가져오기
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
    log.info('제어 모델: F/T로 가상 목표 이동 → D*x_dot = K*(x_virtual - x_command)로 추종')
    log.info('입력 이득 F x/y/z= %.2f mm/(N*s), T rx/ry/rz= %.2f deg/(Nm*s)', VIRTUAL_POINT_FORCE_GAIN, VIRTUAL_POINT_TORQUE_GAIN)
    log.info('K 병진 x/y/z= %.2f N/mm, D 병진 x/y/z= %.2f N*s/mm', STIFFNESS, DAMPING)
    log.info('K 회전 rx/ry/rz=%.3f Nm/deg, D 회전 rx/ry/rz= %.3f Nm*s/deg', ROT_STIFFNESS, ROT_DAMPING)
    log.info('=' * 70)

    # F/T 센서 초기화 및 수신 시작
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

    # 실제 제어 모드일 때만 로봇에 연결
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

    # Bias 측정 전 사용자 확인 대기
    input('\n로봇과 F/T 센서를 정지 상태로 유지한 뒤, Enter를 눌러 Bias를 측정하세요... ')
    bias = measure_bias(sensor)

    if APPLY_ROBOT_COMMANDS:
        log.info('텔레오퍼레이션 모드 시작')
        try:
            indy.start_teleop(2)
            time.sleep(0.5)    # 텔레오퍼레이션 전환 안정화 대기
        except Exception as e:
            log.error('start_teleop 실패: %s', e)
            sensor.stop()
            return

        teleop_state = indy.get_teleop_state()
        log.info('텔레오퍼레이션 상태: %s', teleop_state)

        # op_state 17 = TELE_OP 상태 확인
        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        if op_state != 17:
            log.error('텔레오퍼레이션 전환 실패: op_state=%d, 기대값 17(TELE_OP)', op_state)
            indy.stop_teleop()
            sensor.stop()
            return

    # 제어 상태 초기화
    loop_count = 0
    virtual_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 가상 목표 포즈 (누적)
    command_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 명령 포즈 (누적)
    prev_time = time.time()

    log.info('제어 루프 시작. 종료하려면 Ctrl+C를 누르세요.')

    try:
        while True:
            t_start = time.time()
            dt = t_start - prev_time   # 이전 루프로부터 경과 시간 (초)
            prev_time = t_start

            # dt 스파이크 감지: 허용 최대값을 초과하면 명목 주기로 대체
            if dt > MAX_DT:
                log.debug('[루프 %d] dt 스파이크 %.1fms → %.0fms로 대체',
                          loop_count, dt * 1000, CONTROL_PERIOD * 1000)
                dt = CONTROL_PERIOD

            # 센서에서 원시 F/T 읽기
            ft_raw = sensor.get_ft()
            # Bias 보정 + 불감대 적용
            ft_comp, wrench_eff = compensate_and_deadband(ft_raw, bias, enabled_indices)

            # 가상 목표 포즈 업데이트
            virtual_step = update_virtual_target(virtual_pose, wrench_eff, dt, enabled_indices)
            # 스프링-댐퍼로 명령 Step 계산
            command_step, error = compute_spring_damper_step(
                virtual_pose, command_pose, dt, enabled_indices
            )

            # 명령 포즈에 Step 누적
            for i in range(6):
                command_pose[i] += command_step[i]

            # 상태 로그 출력 (10루프마다)
            log_status(
                ft_raw, ft_comp, wrench_eff, virtual_step, command_step,
                virtual_pose, command_pose, error, loop_count
            )

            if APPLY_ROBOT_COMMANDS:
                # 계산된 명령 포즈를 로봇에 전송
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
                    # 로봇 연결이 비정상이면 루프 탈출
                    if not check_robot_connection(indy):
                        log.error('로봇 연결 비정상. 제어 루프를 종료합니다.')
                        break

            loop_count += 1

            # 남은 시간 슬립으로 제어 주기 유지
            elapsed = time.time() - t_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 처리 시간이 제어 주기를 초과한 경우 경고
                log.warning('[루프 %d] 제어 주기 초과: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, CONTROL_PERIOD * 1000)

    except KeyboardInterrupt:
        log.info('사용자에 의해 중단됨')
    except Exception as e:
        log.error('제어 루프 예외 발생: %s', e, exc_info=True)
    finally:
        # 종료 시 텔레오퍼레이션 모드 해제
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