# ***********************************************************************
#
# test_force_threshold.py
#
# F/T 센서 실시간 외력 Threshold 감지 테스트
#
# 동작:
#   1. CAN 버스로 F/T 센서 연결
#   2. 바이어스 측정 (정지 상태 유지 후 Enter)
#   3. 루프마다 bias-보정 값을 출력하고, 각 축이 threshold 초과 시 WARNING 로그 출력
#
# Threshold 기준 (force_control.py 와 동일):
#   Fx, Fy   : FORCE_THRESHOLD_XY  = 0.5  N
#   Fz       : FORCE_THRESHOLD_Z   = 1.0  N
#   Tx, Ty   : TORQUE_THRESHOLD_RXRY = 0.05 Nm
#   Tz       : TORQUE_THRESHOLD_RZ   = 0.05 Nm
#
# 실행:
#   python test_force_threshold.py
#   Ctrl+C 로 종료
#
# ***********************************************************************

from __future__ import annotations

import logging
import sys
import threading
import time

import can


# ===================================================================
# 하드웨어 설정  (force_control.py 와 동일하게 맞출 것)
# ===================================================================

CAN_INTERFACE = 'slcan'
CAN_CHANNEL   = 'COM3'
CAN_BITRATE   = 1_000_000

CAN_ID_FORCE  = 0x001
CAN_ID_TORQUE = 0x002

# ===================================================================
# Threshold 설정
# ===================================================================

FORCE_THRESHOLD_XY   = 0.5    # N
FORCE_THRESHOLD_Z    = 1.0    # N
TORQUE_THRESHOLD_RXRY = 0.05  # Nm
TORQUE_THRESHOLD_RZ   = 0.05  # Nm

# 바이어스 측정 파라미터
BIAS_SAMPLE_COUNT = 200
BIAS_SAMPLE_DELAY = 0.005     # s

# 출력 주기 (매 N루프마다 현재값 출력)
PRINT_EVERY_N_LOOPS = 5       # 5 * 20ms = 100ms 마다 raw 값 출력


# ===================================================================
# Logging
# ===================================================================

class _CompactFormatter(logging.Formatter):
    LEVEL_ABBREV = {
        logging.DEBUG:   'DEBUG',
        logging.INFO:    'INFO ',
        logging.WARNING: 'WARN ',
        logging.ERROR:   'ERROR',
    }
    def format(self, record):
        ts  = self.formatTime(record, '%H:%M:%S')
        ms  = int(record.msecs)
        lvl = self.LEVEL_ABBREV.get(record.levelno, record.levelname[:5])
        return f'[{ts}.{ms:03d}] {lvl}  {record.getMessage()}'


def _build_logger():
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_CompactFormatter())
    logger = logging.getLogger('FT_ThreshTest')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    logger.propagate = False
    return logger


log = _build_logger()


# ===================================================================
# F/T 센서 리더  (force_control.py 의 FTSensorReader 와 동일)
# ===================================================================

class FTSensorReader:
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
        log.info('CAN 버스 초기화: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._send_start_command()
        self._running = True
        self._thread  = threading.Thread(target=self._recv_loop,
                                         name='FTReader', daemon=True)
        self._thread.start()
        log.info('F/T 센서 수신 시작')

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._bus:
            self._bus.shutdown()
        log.info('F/T 센서 수신 종료')

    def _send_start_command(self):
        data = [0x04, 0x02, 0x06, 0x01, 0x03, 0x01]
        cmd  = can.Message(arbitration_id=0x000, data=data, is_extended_id=False)
        try:
            self._bus.send(cmd)
        except can.CanError as e:
            log.warning('센서 시작 명령 실패 (이미 스트리밍 중이면 무시): %s', e)

    def _recv_loop(self):
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None:
                    log.warning('[CAN] 수신 타임아웃 — 센서 연결 확인')
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
# 바이어스 측정
# ===================================================================

def measure_bias(sensor: FTSensorReader,
                 n_samples: int = BIAS_SAMPLE_COUNT,
                 delay: float   = BIAS_SAMPLE_DELAY) -> list:
    log.info('바이어스 측정 중 (%d 샘플) — 로봇·센서 정지 유지', n_samples)
    accum = [0.0] * 6
    for _ in range(n_samples):
        ft = sensor.get_ft()
        for j in range(6):
            accum[j] += ft[j]
        time.sleep(delay)
    bias = [accum[j] / n_samples for j in range(6)]
    log.info('바이어스 완료: F=[%+.3f, %+.3f, %+.3f] N  '
             'T=[%+.4f, %+.4f, %+.4f] Nm',
             bias[0], bias[1], bias[2], bias[3], bias[4], bias[5])
    return bias


# ===================================================================
# Threshold 감지 및 출력
# ===================================================================

# 각 축의 (threshold, 단위, 표시 이름) 정의
_AXIS_CFG = [
    # (threshold,            unit,  label)
    (FORCE_THRESHOLD_XY,    'N',   'Fx'),
    (FORCE_THRESHOLD_XY,    'N',   'Fy'),
    (FORCE_THRESHOLD_Z,     'N',   'Fz'),
    (TORQUE_THRESHOLD_RXRY, 'Nm',  'Tx'),
    (TORQUE_THRESHOLD_RXRY, 'Nm',  'Ty'),
    (TORQUE_THRESHOLD_RZ,   'Nm',  'Tz'),
]


def check_and_log_thresholds(ft_comp: list, loop_count: int) -> None:
    """
    bias-보정된 6축 wrench 를 threshold 와 비교.
    초과한 축은 WARNING 레벨로 구체적인 값과 함께 출력.
    초과하지 않으면 아무 것도 출력하지 않음.
    """
    exceeded = []
    for i, (thresh, unit, label) in enumerate(_AXIS_CFG):
        val = ft_comp[i]
        if abs(val) >= thresh:
            exceeded.append((label, val, thresh, unit))

    if exceeded:
        parts = ', '.join(
            f'{label}={val:+.3f}{unit} (thresh={thresh:.3f})'
            for label, val, thresh, unit in exceeded
        )
        log.warning('[Loop %4d] *** THRESHOLD EXCEEDED ***  %s', loop_count, parts)


def print_current_values(ft_comp: list, loop_count: int) -> None:
    """현재 bias-보정 값을 정기적으로 INFO 출력."""
    log.info(
        '[Loop %4d]  '
        'Fx=%+6.3fN  Fy=%+6.3fN  Fz=%+6.3fN  |  '
        'Tx=%+7.4fNm  Ty=%+7.4fNm  Tz=%+7.4fNm',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
    )


# ===================================================================
# 메인
# ===================================================================

def main():
    # ── 시작 배너 ────────────────────────────────────────────────────
    log.info('=' * 60)
    log.info('F/T 센서 Threshold 감지 테스트')
    log.info('Threshold: Fx/Fy=%.2fN  Fz=%.2fN  Tx/Ty=%.3fNm  Tz=%.3fNm',
             FORCE_THRESHOLD_XY, FORCE_THRESHOLD_Z,
             TORQUE_THRESHOLD_RXRY, TORQUE_THRESHOLD_RZ)
    log.info('출력 주기: 매 %d 루프 (%.0fms)',
             PRINT_EVERY_N_LOOPS, PRINT_EVERY_N_LOOPS * 20)
    log.info('종료: Ctrl+C')
    log.info('=' * 60)

    # ── 센서 시작 ────────────────────────────────────────────────────
    sensor = FTSensorReader(CAN_INTERFACE, CAN_CHANNEL, CAN_BITRATE)
    try:
        sensor.start()
    except Exception as e:
        log.error('센서 초기화 실패: %s', e)
        return

    log.info('첫 번째 F/T 샘플 대기 중...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T 센서 타임아웃 — 연결 확인 후 재시도')
        sensor.stop()
        return
    log.info('F/T 센서 데이터 확인')

    # ── 바이어스 측정 ────────────────────────────────────────────────
    input('\n로봇·센서를 정지 상태로 두고 Enter 를 눌러 바이어스를 측정하세요... ')
    bias = measure_bias(sensor)

    # ── 실시간 감지 루프 ─────────────────────────────────────────────
    log.info('=' * 60)
    log.info('실시간 감지 시작 — 각 축에 외력을 가해 threshold 초과를 확인하세요')
    log.info('=' * 60)

    loop_count = 0
    LOOP_PERIOD = 0.02   # 20 ms

    try:
        while True:
            t_start = time.time()

            ft_raw  = sensor.get_ft()
            ft_comp = [ft_raw[i] - bias[i] for i in range(6)]

            # 정기 값 출력
            if loop_count % PRINT_EVERY_N_LOOPS == 0:
                print_current_values(ft_comp, loop_count)

            # Threshold 초과 감지 — 매 루프 검사 (즉시 반응)
            check_and_log_thresholds(ft_comp, loop_count)

            loop_count += 1

            elapsed    = time.time() - t_start
            sleep_time = LOOP_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        log.info('사용자 종료 (Ctrl+C)')
    finally:
        sensor.stop()
        log.info('총 루프 수: %d', loop_count)


if __name__ == '__main__':
    main()