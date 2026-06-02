# ***********************************************************************
#
# Author: AIDIN ROBOTICS <info@aidinrobotics.com>
# Modified for uCAN (USB-CAN) by: claude.ai
#
# 이 코드는 AFT200-D80-C 6축 힘/토크 센서에서 CAN 통신으로 데이터를 받아오는 예제
# 하드웨어 Bias 명령(Table 1.7) 추가 버전
#
# 하드웨어 구성:
#   PC --USB-- uCAN --CANH/CANL-- AFT200
#                   별도 5V 전원 공급장치 --VCC/GND-- AFT200
#
# 의존성 설치:
#   pip install python-can
#
# uCAN 장치 유형별 채널 설정:
#
#   [slcan 펌웨어 계열 - CANable, uCAN SLCAN 등]
#     Linux  : interface='slcan', channel='/dev/ttyACM0' (또는 /dev/ttyUSB0)
#     Windows: interface='slcan', channel='COM3'
#
#   [candleLight 펌웨어 / SocketCAN - Linux 전용]
#     interface='socketcan', channel='can0'
#     (사전에 $ sudo ip link set can0 up type can bitrate 1000000 실행 필요)
#
#   [gs_usb 드라이버 - candleLight 기반, Windows/Linux]
#     interface='gs_usb', channel=0
#
# AFT200-D80-C CAN 스펙:
#   - 전원: 5V DC
#   - CAN 속도: 1 Mbps (기본값)
#   - 종단 저항: 버스 양 끝에 120Ω 필요
#   - CAN ID (실제 로그에서 확인된 값):
#       센서 → PC 힘 데이터:  0x001  (Fx, Fy, Fz) / DLC=8, 유효 데이터는 앞 6바이트
#       센서 → PC 토크 데이터: 0x002  (Tx, Ty, Tz) / DLC=8, 유효 데이터는 앞 6바이트
#       뒤 2바이트(data[6], data[7])는 항상 0x00 → 무시
#
# -----------------------------------------------------------------------
#  명령 CAN ID 구조 (매뉴얼 Table 1.x 기준, 명령 전송 대상: 0x102)
#
#   [Table 1.7] Bias(영점) 설정 명령:
#       CAN ID : 0x102
#       data[0]: 센서 ID       (예: 0x01)
#       data[1]: 0x02          (Bias 명령 코드)
#       data[2]: 온도 보상 여부 (0x00: 미포함 / 0x01: 포함)
#       data[3~7]: Don't care  (아무 값이나 가능, 여기서는 0x00 사용)
#
#   [Table 1.8] 연속 전송 시작 명령:
#       CAN ID : 0x102
#       data[0]: 센서 ID       (예: 0x01)
#       data[1]: 0x03          (연속 전송 명령 코드)
#       data[2]: 온도 보상 여부 (0x00: 미포함 / 0x01: 포함)
#       data[3~7]: Don't care
#
# -----------------------------------------------------------------------

import time
import can  # pip install python-can

# ===== uCAN 인터페이스 설정 =====
INTERFACE = 'slcan'      # 'slcan' | 'socketcan' | 'gs_usb'
CHANNEL   = 'COM3'       # Windows: 'COM3', Linux: '/dev/ttyACM0', socketcan: 'can0'
BITRATE   = 1_000_000    # AFT200 기본 CAN 속도: 1 Mbps

# ===== AFT200 CAN ID 정의 =====
SENSOR_ID     = 0x01     # 센서 고유 ID (공장 기본값 0x01, 변경했다면 수정 필요)
CAN_ID_CMD    = 0x102    # PC → 센서: 명령 전송용 CAN ID (매뉴얼 Table 1.7, 1.8)
CAN_ID_FORCE  = 0x001    # 센서 → PC: 힘 데이터   (Fx, Fy, Fz)
CAN_ID_TORQUE = 0x002    # 센서 → PC: 토크 데이터 (Tx, Ty, Tz)

# ===== 온도 보상 설정 =====
# 0x00: 온도 보상 미포함 / 0x01: 온도 보상 포함
TEMP_COMPENSATION = 0x01

# ===== Bias 명령 후 안정화 대기 시간 =====
BIAS_SETTLE_TIME = 0.5   # 초 (센서 내부 Bias 처리 완료 대기)

# ===== 데이터 수신 반복 횟수 =====
LOOP_COUNT = 10_000


# -----------------------------------------------------------------------
# 명령 생성 함수
# -----------------------------------------------------------------------

def build_bias_command() -> can.Message:
    """
    센서 하드웨어에 Bias(영점) 설정 명령을 담은 CAN 메시지를 생성합니다.
    (매뉴얼 Table 1.7)

    센서 내부적으로 현재 측정값을 기준으로 영점을 잡습니다.
    반드시 센서를 정지 상태(무부하)로 유지한 뒤 이 명령을 보내야 합니다.

    CAN ID : 0x102
    data[0]: SENSOR_ID  (센서 ID)
    data[1]: 0x02       (Bias 명령 코드)
    data[2]: 0x00/0x01  (온도 보상 여부)
    data[3~7]: 0x00     (Don't care)
    """
    data = [
        SENSOR_ID,          # [0] 센서 ID
        0x02,               # [1] Bias 명령 코드 (Table 1.7)
        TEMP_COMPENSATION,  # [2] 온도 보상 여부
        0x00,               # [3] Don't care
        0x00,               # [4] Don't care
        0x00,               # [5] Don't care
        0x00,               # [6] Don't care
        0x00,               # [7] Don't care
    ]
    return can.Message(
        arbitration_id=CAN_ID_CMD,
        data=data,
        is_extended_id=False   # 11bit 표준 ID 사용
    )


def build_continuous_start_command() -> can.Message:
    """
    센서에 연속 데이터 전송 시작 명령을 담은 CAN 메시지를 생성합니다.
    (매뉴얼 Table 1.8)

    CAN ID : 0x102
    data[0]: SENSOR_ID  (센서 ID)
    data[1]: 0x03       (연속 전송 명령 코드)
    data[2]: 0x00/0x01  (온도 보상 여부)
    data[3~7]: 0x00     (Don't care)

    참고: 이 센서는 전원 인가 후 명령 없이도 자동으로 데이터를 송출하는
          경우가 있습니다. 명령 전송이 불필요하면 main()에서 생략 가능합니다.
    """
    data = [
        SENSOR_ID,          # [0] 센서 ID
        0x03,               # [1] 연속 전송 명령 코드 (Table 1.8)
        TEMP_COMPENSATION,  # [2] 온도 보상 여부
        0x00,               # [3] Don't care
        0x00,               # [4] Don't care
        0x00,               # [5] Don't care
        0x00,               # [6] Don't care
        0x00,               # [7] Don't care
    ]
    return can.Message(
        arbitration_id=CAN_ID_CMD,
        data=data,
        is_extended_id=False
    )


# -----------------------------------------------------------------------
# 데이터 파싱 함수
# -----------------------------------------------------------------------

def parse_force(msg: can.Message) -> tuple:
    """
    힘 데이터 CAN 프레임을 파싱합니다.
    (매뉴얼 Table 1.9, CAN ID: 센서 ID = 0x001)

    data[0:2] → Fx,  data[2:4] → Fy,  data[4:6] → Fz
    변환식: Force (N) = (raw_value) / 100.0 - 300.0
    """
    d = msg.data
    Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
    Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
    Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
    return Fx, Fy, Fz


def parse_torque(msg: can.Message) -> tuple:
    """
    토크 데이터 CAN 프레임을 파싱합니다.
    (매뉴얼 Table 1.10, CAN ID: 센서 ID + 1 = 0x002)

    data[0:2] → Tx,  data[2:4] → Ty,  data[4:6] → Tz
    변환식: Torque (Nm) = (raw_value) / 500.0 - 50.0
    """
    d = msg.data
    Tx = (d[0] * 256 + d[1]) / 500.0 - 50.0
    Ty = (d[2] * 256 + d[3]) / 500.0 - 50.0
    Tz = (d[4] * 256 + d[5]) / 500.0 - 50.0
    return Tx, Ty, Tz


# -----------------------------------------------------------------------
# 메인 함수
# -----------------------------------------------------------------------

def main():
    print(f"CAN 버스 초기화: interface={INTERFACE}, channel={CHANNEL}, bitrate={BITRATE}")

    with can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=BITRATE) as bus:

        # =================================================================
        # STEP 1: 하드웨어 Bias(영점) 설정 명령 전송 (Table 1.7)
        # -----------------------------------------------------------------
        # 반드시 센서를 정지/무부하 상태로 유지한 뒤 실행해야 합니다.
        # 이 명령으로 센서 내부의 영점이 하드웨어적으로 초기화됩니다.
        # =================================================================
        input("\n[BIAS] 센서를 정지(무부하) 상태로 유지한 뒤 Enter를 누르세요... ")

        bias_cmd = build_bias_command()
        try:
            bus.send(bias_cmd)
            print(
                f"[BIAS] 하드웨어 Bias 명령 전송 완료 "
                f"(CAN ID=0x{CAN_ID_CMD:03X}, "
                f"sensor_id=0x{SENSOR_ID:02X}, "
                f"temp_comp=0x{TEMP_COMPENSATION:02X})"
            )
        except can.CanError as e:
            print(f"[BIAS] Bias 명령 전송 실패: {e}")
            return

        # 센서 내부 Bias 처리가 완료될 때까지 대기
        print(f"[BIAS] 센서 내부 처리 대기 중... ({BIAS_SETTLE_TIME}초)")
        time.sleep(BIAS_SETTLE_TIME)
        print("[BIAS] Bias 설정 완료")

        # =================================================================
        # STEP 2: 연속 데이터 전송 시작 명령 (Table 1.8)
        # -----------------------------------------------------------------
        # Bias 설정 후 연속 전송 시작을 명시적으로 요청합니다.
        # 전원 인가 후 자동으로 데이터를 송출하는 경우에는 생략 가능합니다.
        # =================================================================
        start_cmd = build_continuous_start_command()
        try:
            bus.send(start_cmd)
            print(
                f"[START] 연속 전송 시작 명령 전송 완료 "
                f"(CAN ID=0x{CAN_ID_CMD:03X}, "
                f"sensor_id=0x{SENSOR_ID:02X}, "
                f"temp_comp=0x{TEMP_COMPENSATION:02X})"
            )
        except can.CanError as e:
            # 이미 스트리밍 중인 경우 무시 가능
            print(f"[START] 연속 전송 명령 실패 (이미 스트리밍 중이면 무시 가능): {e}")

        # =================================================================
        # STEP 3: 데이터 수신 루프
        # =================================================================
        Fx = Fy = Fz = 0.0   # 힘 (N)
        Tx = Ty = Tz = 0.0   # 토크 (Nm)
        received_count = 0

        print(f"\n[DATA] 데이터 수신 시작 ({LOOP_COUNT}회)")

        while received_count < LOOP_COUNT:

            # CAN 메시지 수신 (timeout=2.0초)
            msg = bus.recv(timeout=2.0)

            if msg is None:
                print("[경고] 수신 타임아웃 - 센서 연결 및 전원을 확인하세요")
                continue

            # CAN ID로 메시지 유형 판별
            if msg.arbitration_id == CAN_ID_FORCE:
                Fx, Fy, Fz = parse_force(msg)

            elif msg.arbitration_id == CAN_ID_TORQUE:
                Tx, Ty, Tz = parse_torque(msg)

            else:
                # 다른 CAN ID의 메시지는 무시
                continue

            received_count += 1

            # 결과 출력
            print(
                f"Fx : {round(Fx, 2):7.2f}  "
                f"Fy : {round(Fy, 2):7.2f}  "
                f"Fz : {round(Fz, 2):7.2f}  "
                f"Tx : {round(Tx, 2):7.2f}  "
                f"Ty : {round(Ty, 2):7.2f}  "
                f"Tz : {round(Tz, 2):7.2f}"
            )

    # with 블록 종료 시 bus.shutdown() 자동 호출
    print("\nCAN 버스 연결 종료")


if __name__ == "__main__":
    main()