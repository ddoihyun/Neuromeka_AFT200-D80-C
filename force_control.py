# ***********************************************************************
#
# 힘 추종(Force Following) 제어 - 어드미턴스 제어 기반 직접 교시
#
# 시스템 구성:
#   PC --USB-- uCAN --CANH/CANL-- AFT200-D80-C (6축 F/T 센서)
#   PC --Ethernet-- Indy 로봇 (192.168.0.137)
#
# 제어 원리 (순응 제어 - 직접 교시 방식):
#   외부 힘(F_ext) → 0점 보정 → Threshold 필터링
#   → 속도 명령(v = F / B) → 매 루프 증분 이동(누적 방식)
#   → movetelel_rel() 로봇 이동
#
#   ★ 핵심: 힘 → 이번 루프에서의 증분 이동량 (velocity 개념)
#            외력이 풀리면 즉시 정지 (원위치 복귀 없음)
#            → 직접교시(Free-Drive)와 동일한 느낌
#
# 의존성:
#   pip install python-can neuromeka
#
# ***********************************************************************

from __future__ import annotations

import can
import time
import threading
import logging
import sys
from typing import List, Optional

from neuromeka import IndyDCP3, TaskTeleopType
from neuromeka.proto_step import control_msgs_pb2 as control_msgs

# ===================================================================
# [설정] 사용 환경에 맞게 수정
# ===================================================================

# --- CAN (F/T 센서) 설정 ---
CAN_INTERFACE = 'slcan'
CAN_CHANNEL   = 'COM3'
CAN_BITRATE   = 1_000_000

# --- AFT200 CAN ID ---
CAN_ID_FORCE  = 0x001
CAN_ID_TORQUE = 0x002

# --- 로봇 설정 ---
ROBOT_IP    = '192.168.0.137'
ROBOT_INDEX = 0

# ===================================================================
# [제어 파라미터]
# ===================================================================

BIAS_SAMPLE_COUNT  = 200
BIAS_SAMPLE_DELAY  = 0.005

# -----------------------------------------------------------------------
# 힘 Threshold (데드밴드) [N]
#
# ★ Z축 드리프트 원인:
#   중력/케이블 하중이 bias 후에도 ~3~4N 잔류 오프셋으로 남아
#   아무도 안 건드려도 -Z 방향으로 조금씩 이동함.
#   → FORCE_THRESHOLD_Z를 정지 시 COMP Fz 절댓값보다 크게 설정하면 해결.
#   로그 기준: 정지 상태 COMP Fz ≈ -2~-4N  →  5.0N 으로 설정.
#   너무 크면 Z 조작이 둔해지므로 실제 잔류값+1N 정도가 적당.
# -----------------------------------------------------------------------
FORCE_THRESHOLD_XY = 1.0   # X, Y 방향 데드밴드 [N]
FORCE_THRESHOLD_Z  = 5.0   # Z 방향 데드밴드 [N]  ← 중력 잔류 오프셋 흡수

# -----------------------------------------------------------------------
# 순응 제어 댐핑 계수 B [N·s/mm]  →  증분 이동량 = F / B * dt
#   값이 작을수록 같은 힘에 더 빠르게(민감하게) 반응
#   예: B=5  → 5N 인가 시 약 2mm/루프(100mm/s @50Hz)
# -----------------------------------------------------------------------
ADMITTANCE_B_XY = 3.0    # X, Y 방향 댐핑 [N·s/mm]
ADMITTANCE_B_Z  = 3.0    # Z 방향 댐핑 [N·s/mm]

# 한 루프 최대 증분 이동량 클리핑 [mm] (안전)
MAX_STEP_MM = 10.0

# 텔레오퍼레이션 속도/가속도 비율
TEL_VEL_RATIO = 0.25
TEL_ACC_RATIO = 1.0

# 제어 루프 주기 [초]
CONTROL_PERIOD = 0.02   # 50 Hz

# -----------------------------------------------------------------------
# dt 스파이크 상한 [초]
# 네트워크 지연/GC 등으로 dt가 100ms+ 로 튀면 step이 폭발.
# 실제 dt가 이 값을 넘으면 CONTROL_PERIOD 로 고정하여 안전하게 처리.
# -----------------------------------------------------------------------
MAX_DT = CONTROL_PERIOD * 2   # 40ms

# ===================================================================
# 축 이름 (로그 표시용)
# ===================================================================
AXIS_NAMES = ['X(tool)', 'Y(tool)', 'Z(tool)']

# ===================================================================
# 로깅 설정
# ===================================================================
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] %(levelname)s %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger('ForceFollow')


# ===================================================================
# F/T 센서 리더 (백그라운드 스레드)
# ===================================================================

class FTSensorReader(object):
    def __init__(self, interface, channel, bitrate):
        self._interface = interface
        self._channel   = channel
        self._bitrate   = bitrate
        self._lock      = threading.Lock()
        self._Fx = self._Fy = self._Fz = 0.0
        self._Tx = self._Ty = self._Tz = 0.0
        self._running  = False
        self._thread   = None
        self._bus      = None
        self._new_data = threading.Event()

    def start(self):
        log.info('CAN 버스 초기화: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._send_start_command()
        self._running = True
        self._thread  = threading.Thread(target=self._recv_loop, name='FTReader', daemon=True)
        self._thread.start()
        log.info('F/T 센서 수신 스레드 시작')

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._bus is not None:
            self._bus.shutdown()
        log.info('F/T 센서 수신 스레드 종료, CAN 버스 닫힘')

    def _send_start_command(self):
        data = [0x04, 0x02, 0x06, 0x01, 0x03, 0x01]
        cmd  = can.Message(arbitration_id=0x000, data=data, is_extended_id=False)
        try:
            self._bus.send(cmd)
            log.debug('센서 시작 명령 전송 완료')
        except can.CanError as e:
            log.warning('시작 명령 전송 실패 (자동 스트리밍 모드이면 무시): %s', e)

    def _recv_loop(self):
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

    def get_ft(self):
        with self._lock:
            return [self._Fx, self._Fy, self._Fz,
                    self._Tx, self._Ty, self._Tz]

    def wait_for_data(self, timeout=5.0):
        return self._new_data.wait(timeout=timeout)


# ===================================================================
# 0점 보정
# ===================================================================

def measure_bias(sensor, n_samples=BIAS_SAMPLE_COUNT, delay=BIAS_SAMPLE_DELAY):
    log.info('0점 보정 시작: %d 샘플 수집 중... (로봇/센서 정지 상태 유지)', n_samples)
    accum = [0.0] * 6
    for _ in range(n_samples):
        ft = sensor.get_ft()
        for j in range(6):
            accum[j] += ft[j]
        time.sleep(delay)
    bias = [accum[j] / n_samples for j in range(6)]
    log.info('0점 보정 완료: Fx=%.3fN  Fy=%.3fN  Fz=%.3fN  Tx=%.3fNm  Ty=%.3fNm  Tz=%.3fNm',
             bias[0], bias[1], bias[2], bias[3], bias[4], bias[5])
    return bias


# ===================================================================
# 순응 제어 모델: 외력 → 이번 루프 증분 이동량 (velocity * dt)
#
# ★ 변경 핵심:
#   기존 어드미턴스:  disp = F / K  (스프링 모델 → 힘 풀면 원위치)
#   순응 제어:        step = F / B * dt  (댐퍼 모델 → 힘 풀면 정지)
#
# movetelel_rel(RELATIVE 모드)는 start_teleop 시점 기준 상대 위치를
# 목표로 삼으므로, 매 루프 증분(step)을 누적하여 전달해야
# 직접교시처럼 동작합니다.
# ===================================================================

def compute_step(ft_compensated, dt):
    # type: (List[float], float) -> List[float]
    """
    보정된 힘 → 이번 루프 이동 증분(mm)

    반환값: [step_x, step_y, step_z, 0, 0, 0]  (tool 좌표계 기준)
    """
    Fx, Fy, Fz = ft_compensated[0], ft_compensated[1], ft_compensated[2]

    def deadband(val, threshold):
        if abs(val) < threshold:
            return 0.0
        return (val - threshold) if val > 0 else (val + threshold)

    Fx_eff = deadband(Fx, FORCE_THRESHOLD_XY)
    Fy_eff = deadband(Fy, FORCE_THRESHOLD_XY)
    Fz_eff = deadband(Fz, FORCE_THRESHOLD_Z)

    # 댐퍼 모델: step = (F / B) * dt  [mm]
    sx = (Fx_eff / ADMITTANCE_B_XY) * dt
    sy = (Fy_eff / ADMITTANCE_B_XY) * dt
    sz = (Fz_eff / ADMITTANCE_B_Z)  * dt

    def clip(val, limit):
        return max(-limit, min(limit, val))

    sx = clip(sx, MAX_STEP_MM)
    sy = clip(sy, MAX_STEP_MM)
    sz = clip(sz, MAX_STEP_MM)

    return [sx, sy, sz, 0.0, 0.0, 0.0]


# ===================================================================
# 로봇 상태 확인
# ===================================================================

def check_robot_connection(indy):
    try:
        robot_data = indy.get_robot_data()
        op_state   = robot_data.get('op_state', -1)
        sim_mode   = robot_data.get('sim_mode', False)
        log.info('로봇 연결 확인: op_state=%d, sim_mode=%s', op_state, sim_mode)
        ABNORMAL = {0, 2, 3, 8, 15}
        if op_state in ABNORMAL:
            log.error('로봇 비정상 상태: op_state=%d', op_state)
            return False
        if sim_mode:
            log.warning('시뮬레이션 모드 활성화 - 실제 로봇이 움직이지 않습니다.')
        return True
    except Exception as e:
        log.error('로봇 연결 확인 실패: %s', e)
        return False


# ===================================================================
# 상태 로그 출력 (10루프마다)
# ===================================================================

def log_status(ft_raw, ft_comp, step, cum_disp, loop_count):
    # type: (List[float], List[float], List[float], List[float], int) -> None
    if loop_count % 10 != 0:
        return

    # 실제 이동 중인 축 표시
    moving_axes = []
    for i, name in enumerate(AXIS_NAMES):
        if abs(step[i]) > 1e-4:
            direction = '+' if step[i] > 0 else '-'
            moving_axes.append('{}{} {:.2f}mm'.format(direction, name, abs(step[i])))
    move_str = ', '.join(moving_axes) if moving_axes else '정지'

    log.debug(
        '[Loop %4d] RAW F=[%+6.2f, %+6.2f, %+6.2f]N  '
        'COMP F=[%+6.2f, %+6.2f, %+6.2f]N  '
        '이번 증분=[%+6.3f, %+6.3f, %+6.3f]mm  '
        '누적 변위=[%+7.2f, %+7.2f, %+7.2f]mm  '
        '이동 축: %s',
        loop_count,
        ft_raw[0],  ft_raw[1],  ft_raw[2],
        ft_comp[0], ft_comp[1], ft_comp[2],
        step[0],    step[1],    step[2],
        cum_disp[0], cum_disp[1], cum_disp[2],
        move_str,
    )


# ===================================================================
# 메인 제어 루프
# ===================================================================

def main():
    log.info('=' * 60)
    log.info('힘 추종 직접 교시 시스템 시작')
    log.info('제어 방식: 순응 제어 (Compliant / Direct Teaching)')
    log.info('F_threshold_xy=%.1fN, F_threshold_z=%.1fN, B_xy=%.1fN·s/mm, B_z=%.1fN·s/mm, max_step=%.1fmm',
             FORCE_THRESHOLD_XY, FORCE_THRESHOLD_Z, ADMITTANCE_B_XY, ADMITTANCE_B_Z, MAX_STEP_MM)
    log.info('힘 풀면 즉시 정지 (원위치 복귀 없음) - 직접교시 모드와 동일')
    log.info('=' * 60)

    # ------------------------------------------------------------------
    # 1. F/T 센서 초기화
    # ------------------------------------------------------------------
    sensor = FTSensorReader(CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE)
    try:
        sensor.start()
    except Exception as e:
        log.error('F/T 센서 초기화 실패: %s', e)
        return

    log.info('첫 F/T 데이터 수신 대기 중...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T 센서 데이터 수신 타임아웃!')
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
    # 3. 영점 보정
    # ------------------------------------------------------------------
    input('\n[준비] 로봇과 센서를 정지 상태로 두고 Enter를 누르면 영점 보정을 시작합니다... ')
    bias = measure_bias(sensor)

    # ------------------------------------------------------------------
    # 4. 텔레오퍼레이션 시작
    # ------------------------------------------------------------------
    log.info('텔레오퍼레이션 모드 시작 (TaskTeleopType.RELATIVE - tool 좌표계 기준)')
    try:
        indy.start_teleop(2)
        time.sleep(0.5)
    except Exception as e:
        log.error('텔레오퍼레이션 시작 실패: %s', e)
        sensor.stop()
        return

    teleop_state = indy.get_teleop_state()
    log.info('텔레오퍼레이션 상태: %s', teleop_state)

    robot_data = indy.get_robot_data()
    op_state   = robot_data.get('op_state', -1)
    if op_state != 17:
        log.error('텔레오퍼레이션 전환 실패! op_state=%d (기대값: 17=TELE_OP)', op_state)
        log.error('Conty에서 로봇이 IDLE 상태인지, 에러가 없는지 확인하세요.')
        indy.stop_teleop()
        sensor.stop()
        return
    log.info('텔레오퍼레이션 전환 성공: op_state=17 (TELE_OP) 확인')

    # ------------------------------------------------------------------
    # 5. 순응 제어 루프
    #
    # ★ RELATIVE 모드 동작 방식:
    #   movetelel_rel(tpos) → start_teleop 시점의 위치 + tpos 로 이동
    #
    # ★ 직접교시처럼 동작시키는 방법:
    #   매 루프마다 (F/B)*dt 만큼의 증분(step)을 계산하고
    #   cumulative_disp(누적 변위)에 더하여 전달
    #   → 힘이 없으면 step=0 → 누적 변위 유지 → 현재 위치에서 정지
    #   → 힘이 있으면 step≠0 → 누적 변위 증가 → 해당 방향으로 이동
    # ------------------------------------------------------------------
    log.info('순응 제어 루프 시작. Ctrl+C로 종료.')
    log.info('축 매핑: tool X(앞뒤), tool Y(좌우), tool Z(상하)')

    loop_count  = 0
    # start_teleop 기준 누적 변위 [mm] - RELATIVE 모드 목표 위치
    cum_disp    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    prev_time   = time.time()

    try:
        while True:
            t_start  = time.time()
            dt       = t_start - prev_time
            prev_time = t_start

            # dt 스파이크 방지: 네트워크 지연 등으로 dt가 튀면 step 폭발 가능
            # → MAX_DT(40ms) 초과 시 CONTROL_PERIOD 로 고정
            if dt > MAX_DT:
                log.debug('[Loop %d] dt 스파이크 감지 (%.1fms) → %.0fms로 고정',
                          loop_count, dt * 1000, CONTROL_PERIOD * 1000)
                dt = CONTROL_PERIOD

            # 5-1. F/T 읽기 + 0점 보정
            ft_raw  = sensor.get_ft()
            ft_comp = [ft_raw[i] - bias[i] for i in range(6)]

            # 5-2. 이번 루프 증분 계산 (댐퍼 모델)
            step = compute_step(ft_comp, dt)

            # 5-3. 누적 변위 업데이트 (RELATIVE 모드의 목표 위치)
            for i in range(6):
                cum_disp[i] += step[i]

            # 5-4. 상태 로그 (10루프마다, 실제 이동 축 표시)
            log_status(ft_raw, ft_comp, step, cum_disp[:3], loop_count)

            # 5-5. 로봇 이동 명령 (TCP 좌표계 기준)
            #   TELE_TASK_TCP: tool 좌표계 기준 증분 이동
            #   → 로봇 자세가 바뀌어도 항상 tool 방향으로 이동
            try:
                indy.control.MoveTeleL(
                    control_msgs.MoveTeleLReq(
                        tpos=cum_disp,
                        vel_ratio=TEL_VEL_RATIO,
                        acc_ratio=TEL_ACC_RATIO,
                        method=control_msgs.TELE_TASK_TCP,
                    )
                )
            except Exception as e:
                log.error('MoveTeleL(TCP) 오류: %s', e)
                if not check_robot_connection(indy):
                    log.error('로봇 연결 이상 - 제어 루프 중단')
                    break

            loop_count += 1

            # 5-6. 주기 맞춤
            elapsed    = time.time() - t_start
            sleep_time = CONTROL_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning('[Loop %d] 제어 주기 초과: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, CONTROL_PERIOD * 1000)

    except KeyboardInterrupt:
        log.info('사용자 중단 (Ctrl+C)')

    except Exception as e:
        log.error('제어 루프 예외: %s', e, exc_info=True)

    finally:
        log.info('텔레오퍼레이션 종료 중...')
        try:
            indy.stop_teleop()
            time.sleep(0.3)
            log.info('텔레오퍼레이션 종료 완료')
        except Exception as e:
            log.error('stop_teleop 오류: %s', e)

        sensor.stop()
        log.info('시스템 종료 완료. 총 제어 루프 수: %d', loop_count)
        log.info('최종 누적 변위: X(tool)=%.2fmm, Y(tool)=%.2fmm, Z(tool)=%.2fmm',
                 cum_disp[0], cum_disp[1], cum_disp[2])


# ===================================================================
if __name__ == '__main__':
    main()