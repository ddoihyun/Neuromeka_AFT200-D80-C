# ***********************************************************************
#
# Author: AIDIN ROBOTICS <info@aidinrobotics.com>
# Modified for uCAN (USB-CAN) by: claude.ai
#
# 이 코드는 AFT200-D80-C 6축 힘/토크 센서에서 CAN 통신으로 데이터를 받아오는 예제
#
# 하드웨어 구성:
#   PC --USB-- uCAN --CANH/CANL-- AFT200
#                   별도 5V 전원 공급장치 --VCC/GND-- AFT200
#
# 의존성 설치:
#   pip install python-can
#
# uCAN 장치 유형별 채널 설정 (아래 INTERFACE / CHANNEL 참고):
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
# ***********************************************************************

import can  # pip install python-can

# ===== uCAN 인터페이스 설정 =====
# 사용 중인 장치/OS에 맞게 아래 두 값을 수정하세요
INTERFACE = 'slcan'    # 'slcan' | 'socketcan' | 'gs_usb'
CHANNEL   = 'COM3'    # Windows: 'COM3', Linux: '/dev/ttyACM0', socketcan: 'can0'
BITRATE   = 1_000_000  # AFT200 기본 CAN 속도: 1 Mbps

# ===== AFT200 CAN ID 정의 (실제 로그에서 확인된 값) =====
CAN_ID_FORCE  = 0x001  # 센서 → PC: 힘 데이터   (Fx, Fy, Fz)
CAN_ID_TORQUE = 0x002  # 센서 → PC: 토크 데이터 (Tx, Ty, Tz)

# ===== 데이터 수신 반복 횟수 =====
LOOP_COUNT = 10_000


def build_start_command() -> can.Message:
    """
    센서에 '연속 데이터 전송 시작' 명령을 담은 CAN 메시지를 생성합니다.

    로그 확인 결과, 이 센서는 전원 인가 후 명령 없이도 자동으로
    0x001/0x002 ID로 데이터를 송출합니다.
    명령 전송이 필요 없는 경우 main()에서 이 호출을 생략해도 됩니다.

    원본 TCP 패킷 구조 대응:
        TCP sendData[0]  = 0x04  → CAN data[0] (헤더)
        TCP sendData[3]  = 0x02  → CAN data[1] (명령 타입)
        TCP sendData[4]  = 0x06  → CAN data[2] (데이터 길이)
        TCP sendData[5]  = 0x01  → CAN data[3] (센서 ID)
        TCP sendData[6]  = 0x03  → CAN data[4] (전송 모드)
        TCP sendData[7]  = 0x01  → CAN data[5] (온도 보정 활성화)
    """
    data = [
        0x04,  # 헤더
        0x02,  # 명령 타입: 데이터 스트리밍 시작
        0x06,  # 페이로드 길이
        0x01,  # 센서 ID
        0x03,  # 전송 모드 (연속 전송)
        0x01,  # 온도 보정 활성화
    ]
    return can.Message(
        arbitration_id=0x000,  # 명령용 CAN ID (센서 매뉴얼 확인 필요)
        data=data,
        is_extended_id=False   # 11bit 표준 ID 사용
    )


def parse_force(msg: can.Message) -> tuple:
    """
    힘 데이터 CAN 프레임을 파싱합니다. (원본 TCP recvData[4]==0x01 케이스)

    CAN data[0:2] → Fx,  data[2:4] → Fy,  data[4:6] → Fz
    변환식: value = (HIGH*256 + LOW) / 100.0 - 300.0   [단위: N]
    """
    d = msg.data
    Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
    Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
    Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
    return Fx, Fy, Fz


def parse_torque(msg: can.Message) -> tuple:
    """
    토크 데이터 CAN 프레임을 파싱합니다. (원본 TCP recvData[4]==0x02 케이스)

    CAN data[0:2] → Tx,  data[2:4] → Ty,  data[4:6] → Tz
    변환식: value = (HIGH*256 + LOW) / 500.0 - 50.0   [단위: Nm]
    """
    d = msg.data
    Tx = (d[0] * 256 + d[1]) / 500.0 - 50.0
    Ty = (d[2] * 256 + d[3]) / 500.0 - 50.0
    Tz = (d[4] * 256 + d[5]) / 500.0 - 50.0
    return Tx, Ty, Tz


def main():
    # ===== CAN 버스 초기화 =====
    print(f"CAN 버스 초기화: interface={INTERFACE}, channel={CHANNEL}, bitrate={BITRATE}")

    with can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=BITRATE) as bus:

        # ===== 센서 초기 명령 전송 (데이터 스트리밍 시작) =====
        cmd = build_start_command()
        bus.send(cmd)
        print("센서 시작 명령 전송 완료")

        # ===== 변수 초기화 =====
        Fx = Fy = Fz = 0.0  # 힘 (N)
        Tx = Ty = Tz = 0.0  # 토크 (Nm)

        received_count = 0

        # ===== 데이터 수신 루프 =====
        # 원본: for i in range(10000): recvData = recvMsg()
        while received_count < LOOP_COUNT:

            # CAN 메시지 수신 (timeout=2.0초 - 원본 소켓 타임아웃과 동일)
            msg = bus.recv(timeout=2.0)

            if msg is None:
                print("[경고] 수신 타임아웃 - 센서 연결 및 전원을 확인하세요")
                continue

            # ===== CAN ID로 메시지 유형 판별 =====
            # 원본: recvData[4] == 0x01 → 힘 / 0x02 → 토크
            # CAN에서는 arbitration_id로 구분

            if msg.arbitration_id == CAN_ID_FORCE:
                # 힘 데이터 프레임
                Fx, Fy, Fz = parse_force(msg)

            elif msg.arbitration_id == CAN_ID_TORQUE:
                # 토크 데이터 프레임
                Tx, Ty, Tz = parse_torque(msg)

            else:
                # 다른 CAN ID의 메시지는 무시 (버스에 다른 장치가 있을 경우)
                continue

            received_count += 1

            # ===== 결과 출력 (원본과 동일한 형식) =====
            print(
                f"Fx : {round(Fx, 2)} "
                f"Fy : {round(Fy, 2)} "
                f"Fz : {round(Fz, 2)} "
                f"Tx : {round(Tx, 2)} "
                f"Ty : {round(Ty, 2)} "
                f"Tz : {round(Tz, 2)} "
            )

    # with 블록 종료 시 bus.shutdown() 자동 호출 (소켓의 s.close()에 해당)
    print("CAN 버스 연결 종료")


if __name__ == "__main__":
    main()