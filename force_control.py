# ***********************************************************************
#
# 힘 추종(Force Following) 제어 - 어드미턴스 제어 기반 직접 교시
#
# 시스템 구성:
#   PC --USB-- uCAN --CANH/CANL-- AFT200-D80-C (6축 F/T 센서)
#   PC --Ethernet-- Indy 로봇 (192.168.0.161)
#
# 제어 원리 (어드미턴스 제어):
#   외부 힘(F_ext) → 0점 보정 → 중력 보상 → Threshold 필터링
#   → 어드미턴스 모델 (F = kx) → tool좌표계 기준 상대 위치 명령
#   → movetelel_rel() 로봇 이동
#
# 의존성:
#   pip install python-can neuromeka
#
# 하드웨어:
#   - AFT200-D80-C 6축 F/T 센서 (CAN 1Mbps)
#   - Neuromeka Indy 로봇 (IndyDCP3)
#   - uCAN (slcan 또는 socketcan)
#
# Python 3.7 호환
# ***********************************************************************

from __future__ import annotations

import can
import time
import threading
import logging
import sys
from typing import List, Optional

from neuromeka import IndyDCP3, TaskTeleopType

# ===================================================================
# [설정] 사용 환경에 맞게 수정
# ===================================================================

# --- CAN (F/T 센서) 설정 ---
CAN_INTERFACE = 'slcan'          # 'slcan' | 'socketcan' | 'gs_usb'
CAN_CHANNEL   = 'COM3'           # Windows: 'COM3' / Linux: '/dev/ttyACM0' / socketcan: 'can0'
CAN_BITRATE   = 1_000_000        # AFT200 기본값: 1 Mbps

# --- AFT200 CAN ID ---
CAN_ID_FORCE  = 0x001            # 힘 데이터  (Fx, Fy, Fz)  [N]
CAN_ID_TORQUE = 0x002            # 토크 데이터 (Tx, Ty, Tz) [Nm]

# --- 로봇 설정 ---
ROBOT_IP    = '192.168.0.161'
ROBOT_INDEX = 0

# ===================================================================
# [제어 파라미터]
# ===================================================================

# 0점 보정(바이어스) 샘플 수 및 대기 시간
BIAS_SAMPLE_COUNT = 200          # 보정 샘플 수
BIAS_SAMPLE_DELAY = 0.005        # 샘플 간격 (초)

# 힘 Threshold [N] - 이 값 이상의 힘에서만 로봇이 반응
FORCE_THRESHOLD_N  = 1.0         # 힘 임계값 [N]
TORQUE_THRESHOLD_NM = 0.3        # 토크 임계값 [Nm] (현재 미사용, 위치 제어에만 힘 사용)

# 어드미턴스 이득 (F = k * x  →  x = F / k)
# 단위: [mm/N] - k 값이 클수록 같은 힘에 대해 더 작게 움직임
ADMITTANCE_K_XY = 1.0           # X, Y 방향 강성 [N/mm]
ADMITTANCE_K_Z  = 1.0           # Z 방향 강성 [N/mm] (수직 방향 더 단단하게)

# 출력 위치 변위 클리핑 [mm] - 1 제어 주기 최대 이동량
MAX_DISP_MM = 100.0                # 안전을 위한 최대 변위

# 텔레오퍼레이션 속도/가속도 비율
# movetelel_rel 기준: vel_ratio/acc_ratio 범위는 0.0 ~ 1.0
TEL_VEL_RATIO = 1.0
TEL_ACC_RATIO = 1.0

# 제어 루프 주기 [초]
CONTROL_PERIOD = 0.02            # 50 Hz

# ===================================================================
# 로깅 설정
# ===================================================================
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
log = logging.getLogger('ForceFollow')


# ===================================================================
# F/T 센서 리더 (백그라운드 스레드)
# ===================================================================

class FTSensorReader(object):
    """
    별도 스레드로 CAN 버스를 수신하여 F/T 데이터를 갱신한다.
    메인 루프는 .get_ft() 를 폴링하여 최신 값을 읽는다.
    """

    def __init__(self, interface, channel, bitrate):
        # type: (str, str, int) -> None
        self._interface = interface
        self._channel   = channel
        self._bitrate   = bitrate

        self._lock = threading.Lock()
        self._Fx = self._Fy = self._Fz = 0.0
        self._Tx = self._Ty = self._Tz = 0.0

        self._running = False
        self._thread  = None  # type: Optional[threading.Thread]
        self._bus     = None  # type: Optional[can.BusABC]

        self._new_data = threading.Event()

    # ------------------------------------------------------------------
    def start(self):
        # type: () -> None
        log.info('CAN 버스 초기화: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate
        )
        self._send_start_command()

        self._running = True
        self._thread  = threading.Thread(target=self._recv_loop, name='FTReader', daemon=True)
        self._thread.start()
        log.info('F/T 센서 수신 스레드 시작')

    def stop(self):
        # type: () -> None
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._bus is not None:
            self._bus.shutdown()
        log.info('F/T 센서 수신 스레드 종료, CAN 버스 닫힘')

    # ------------------------------------------------------------------
    def _send_start_command(self):
        # type: () -> None
        """AFT200 연속 스트리밍 시작 명령 전송."""
        data = [0x04, 0x02, 0x06, 0x01, 0x03, 0x01]
        cmd = can.Message(arbitration_id=0x000, data=data, is_extended_id=False)
        try:
            self._bus.send(cmd)
            log.debug('센서 시작 명령 전송 완료')
        except can.CanError as e:
            log.warning('시작 명령 전송 실패 (자동 스트리밍 모드이면 무시): %s', e)

    def _recv_loop(self):
        # type: () -> None
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None:
                    log.warning('[CAN] 수신 타임아웃 - 센서 연결 확인 필요')
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
                    log.error('[CAN] 수신 오류: %s', e)

    # ------------------------------------------------------------------
    def get_ft(self):
        # type: () -> List[float]
        """현재 F/T 값 반환: [Fx, Fy, Fz, Tx, Ty, Tz]"""
        with self._lock:
            return [self._Fx, self._Fy, self._Fz,
                    self._Tx, self._Ty, self._Tz]

    def wait_for_data(self, timeout=5.0):
        # type: (float) -> bool
        """첫 데이터가 들어올 때까지 대기. 성공 시 True 반환."""
        return self._new_data.wait(timeout=timeout)


# ===================================================================
# 0점 보정 (바이어스 측정)
# ===================================================================

def measure_bias(sensor, n_samples=BIAS_SAMPLE_COUNT, delay=BIAS_SAMPLE_DELAY):
    # type: (FTSensorReader, int, float) -> List[float]
    """
    n_samples 동안 F/T 데이터를 샘플링하여 평균(바이어스)을 계산한다.
    로봇이 정지 상태에서 호출해야 한다.
    """
    log.info('0점 보정 시작: %d 샘플 수집 중... (로봇/센서 정지 상태 유지)', n_samples)
    accum = [0.0] * 6
    for i in range(n_samples):
        ft = sensor.get_ft()
        for j in range(6):
            accum[j] += ft[j]
        time.sleep(delay)

    bias = [accum[j] / n_samples for j in range(6)]
    log.info('0점 보정 완료: Fx_bias=%.3fN, Fy_bias=%.3fN, Fz_bias=%.3fN '
             'Tx_bias=%.3fNm, Ty_bias=%.3fNm, Tz_bias=%.3fNm',
             bias[0], bias[1], bias[2], bias[3], bias[4], bias[5])
    return bias


# ===================================================================
# 어드미턴스 모델: 외력 → 위치 변위
# ===================================================================

def compute_displacement(ft_compensated):
    # type: (List[float]) -> List[float]
    """
    보정된 F/T 값을 어드미턴스 모델로 위치 변위(mm)로 변환.

    - Threshold 미만의 힘은 0으로 처리 (데드밴드)
    - F = k * x  →  x = F / k
    - tool 좌표계 기준 [dx, dy, dz, du, dv, dw]
      (현재 구현: 위치 3축만 사용, 자세 변화 없음)
    """
    Fx, Fy, Fz = ft_compensated[0], ft_compensated[1], ft_compensated[2]

    # --- Threshold 적용 (데드밴드) ---
    def deadband(val, threshold):
        # type: (float, float) -> float
        if abs(val) < threshold:
            return 0.0
        # 임계값 초과분만 사용 (부드러운 전환)
        return (val - threshold) if val > 0 else (val + threshold)

    Fx_eff = deadband(Fx, FORCE_THRESHOLD_N)
    Fy_eff = deadband(Fy, FORCE_THRESHOLD_N)
    Fz_eff = deadband(Fz, FORCE_THRESHOLD_N)

    # --- 어드미턴스 변환 (x = F / k) ---
    dx = Fx_eff / ADMITTANCE_K_XY
    dy = Fy_eff / ADMITTANCE_K_XY
    dz = Fz_eff / ADMITTANCE_K_Z

    # --- 최대 변위 클리핑 (안전) ---
    def clip(val, limit):
        # type: (float, float) -> float
        return max(-limit, min(limit, val))

    dx = clip(dx, MAX_DISP_MM)
    dy = clip(dy, MAX_DISP_MM)
    dz = clip(dz, MAX_DISP_MM)

    return [dx, dy, dz, 0.0, 0.0, 0.0]


# ===================================================================
# 로봇 상태 확인 유틸리티
# ===================================================================

def check_robot_connection(indy):
    # type: (IndyDCP3) -> bool
    """로봇 연결 및 기본 상태를 확인한다."""
    try:
        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        sim_mode = robot_data.get('sim_mode', False)

        log.info('로봇 연결 확인: op_state=%d, sim_mode=%s', op_state, sim_mode)

        # op_state: IDLE=5, MOVING=6, TELE_OP=17 등 정상 범위 확인
        # 0: SYSTEM_OFF, 2: VIOLATE, 8: COLLISION 등은 비정상
        ABNORMAL_STATES = {0, 2, 3, 8, 15}  # SYSTEM_OFF, VIOLATE, COLLISION 등
        if op_state in ABNORMAL_STATES:
            log.error('로봇 비정상 상태: op_state=%d. 제어 중단.', op_state)
            return False

        if sim_mode:
            log.warning('시뮬레이션 모드 활성화 - 실제 로봇이 움직이지 않습니다.')

        return True
    except Exception as e:
        log.error('로봇 연결 확인 실패: %s', e)
        return False


def log_status(ft_raw, ft_comp, disp, loop_count):
    # type: (List[float], List[float], List[float], int) -> None
    """현재 상태를 주기적으로 로그 출력 (10 루프마다)."""
    if loop_count % 10 != 0:
        return
    log.debug(
        '[Loop %4d] RAW F=[%5.2f, %5.2f, %5.2f]N  '
        'COMP F=[%5.2f, %5.2f, %5.2f]N  '
        'DISP=[%5.3f, %5.3f, %5.3f]mm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_comp[0], ft_comp[1], ft_comp[2],
        disp[0], disp[1], disp[2]
    )


# ===================================================================
# 메인 제어 루프
# ===================================================================

def main():
    # type: () -> None
    log.info('=' * 60)
    log.info('힘 추종 직접 교시 시스템 시작')
    log.info('제어 방식: 어드미턴스 제어 (Force Following)')
    log.info('F_threshold=%.1fN, K_xy=%.1fN/mm, K_z=%.1fN/mm, max_disp=%.1fmm',
             FORCE_THRESHOLD_N, ADMITTANCE_K_XY, ADMITTANCE_K_Z, MAX_DISP_MM)
    log.info('=' * 60)

    # ------------------------------------------------------------------
    # 1. F/T 센서 초기화
    # ------------------------------------------------------------------
    sensor = FTSensorReader(CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE)
    try:
        sensor.start()
    except Exception as e:
        log.error('F/T 센서 초기화 실패: %s', e)
        log.error('CAN 인터페이스 설정(CAN_INTERFACE, CAN_CHANNEL)을 확인하세요.')
        return

    log.info('첫 F/T 데이터 수신 대기 중...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T 센서 데이터 수신 타임아웃! 센서 연결을 확인하세요.')
        sensor.stop()
        return
    log.info('F/T 센서 데이터 수신 확인')

    # ------------------------------------------------------------------
    # 2. 로봇 연결
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 3. 영점 보정 (바이어스 측정)
    #    로봇은 정지 상태, 사람은 센서에 힘을 가하지 않은 상태여야 함
    # ------------------------------------------------------------------
    input('\n[준비] 로봇과 센서를 정지 상태로 두고 Enter를 누르면 영점 보정을 시작합니다... ')
    bias = measure_bias(sensor)

    # ------------------------------------------------------------------
    # 4. 텔레오퍼레이션 시작
    # ------------------------------------------------------------------
    log.info('텔레오퍼레이션 모드 시작 (TaskTeleopType.RELATIVE - tool 좌표계 기준)')
    try:
        indy.start_teleop(method=TaskTeleopType.RELATIVE)
        time.sleep(0.5)
    except Exception as e:
        log.error('텔레오퍼레이션 시작 실패: %s', e)
        sensor.stop()
        return

    teleop_state = indy.get_teleop_state()
    log.info('텔레오퍼레이션 상태: %s', teleop_state)

    # op_state가 TELE_OP(17)인지 확인
    robot_data = indy.get_robot_data()
    op_state = robot_data.get('op_state', -1)
    if op_state != 17:
        log.error('텔레오퍼레이션 전환 실패! op_state=%d (기대값: 17=TELE_OP)', op_state)
        log.error('Conty에서 로봇이 IDLE 상태인지, 에러가 없는지 확인하세요.')
        indy.stop_teleop()
        sensor.stop()
        return
    log.info('텔레오퍼레이션 전환 성공: op_state=17 (TELE_OP) 확인')

    # ------------------------------------------------------------------
    # 5. 힘 추종 제어 루프
    # ------------------------------------------------------------------
    log.info('힘 추종 제어 루프 시작. Ctrl+C로 종료.')
    loop_count = 0

    try:
        while True:
            t_start = time.time()

            # 5-1. F/T 데이터 읽기
            ft_raw = sensor.get_ft()

            # 5-2. 0점 보정 적용 (바이어스 제거)
            ft_comp = [ft_raw[i] - bias[i] for i in range(6)]

            # 5-3. 어드미턴스 모델: 외력 → 위치 변위
            disp = compute_displacement(ft_comp)

            # 5-4. 상태 로그 출력
            log_status(ft_raw, ft_comp, disp, loop_count)

            # 5-5. 변위가 있을 때만 로봇 이동 명령 전송
            #      (모두 0이면 굳이 통신하지 않아도 되지만, 텔레오퍼레이션
            #       특성상 명령을 지속 전송하는 게 더 안정적)
            try:
                indy.movetelel_rel(
                    tpos=disp,
                    vel_ratio=TEL_VEL_RATIO,
                    acc_ratio=TEL_ACC_RATIO
                )
            except Exception as e:
                log.error('movetelel_rel 오류: %s', e)
                # 로봇 상태 재확인 후 계속 시도
                if not check_robot_connection(indy):
                    log.error('로봇 연결 이상 - 제어 루프 중단')
                    break

            loop_count += 1

            # 5-6. 주기 맞춤 (정확한 CONTROL_PERIOD 유지)
            elapsed = time.time() - t_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning('[Loop %d] 제어 주기 초과: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, CONTROL_PERIOD * 1000)

    except KeyboardInterrupt:
        log.info('사용자 중단 (Ctrl+C)')

    except Exception as e:
        log.error('제어 루프 예외 발생: %s', e, exc_info=True)

    finally:
        # ------------------------------------------------------------------
        # 6. 정리
        # ------------------------------------------------------------------
        log.info('텔레오퍼레이션 종료 중...')
        try:
            indy.stop_teleop()
            time.sleep(0.3)
            log.info('텔레오퍼레이션 종료 완료')
        except Exception as e:
            log.error('stop_teleop 오류: %s', e)

        sensor.stop()
        log.info('시스템 종료 완료. 총 제어 루프 수: %d', loop_count)


# ===================================================================
# 진입점
# ===================================================================
if __name__ == '__main__':
    main()