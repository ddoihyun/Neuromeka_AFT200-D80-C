import can
import time

# COM 포트 번호를 본인 환경에 맞게 수정하세요
COM_PORT = 'COM3'
BITRATE = 1000000  # 1Mbps

# 센서 CAN ID 설정 (기본값 0x01)
SENSOR_ID = 0x01
CMD_ID = 0x100 + SENSOR_ID  # → 0x101

def decode_force(high, low):
    return (high * 256 + low) / 100.0 - 300.0

def decode_torque(high, low):
    return (high * 256 + low) / 500.0 - 50.0

def main():
    # slcan 인터페이스로 연결
    bus = can.Bus(
        interface='slcan',
        channel=COM_PORT,
        bitrate=BITRATE,
        sleep_after_open=2.0
    )

    print("Connected. Sending start command...")

    # AFT200 연속 전송 모드 시작 명령
    # CAN Msg [ID=0x102, data: 0x01, 0x03, 0x01]
    start_msg = can.Message(
        arbitration_id=0x102,
        data=[0x01, 0x03, 0x01],
        is_extended_id=False
    )
    bus.send(start_msg)
    time.sleep(0.1)

    Fx = Fy = Fz = Tx = Ty = Tz = 0.0

    print("Reading sensor data (Ctrl+C to stop)...")
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                print("Timeout: no message received")
                continue

            # 힘 데이터 (CAN ID = 센서ID + 0x000)
            if msg.arbitration_id == SENSOR_ID:
                Fx = decode_force(msg.data[0], msg.data[1])
                Fy = decode_force(msg.data[2], msg.data[3])
                Fz = decode_force(msg.data[4], msg.data[5])

            # 토크 데이터 (CAN ID = 센서ID + 0x001 → 0x02)
            elif msg.arbitration_id == SENSOR_ID + 1:
                Tx = decode_torque(msg.data[0], msg.data[1])
                Ty = decode_torque(msg.data[2], msg.data[3])
                Tz = decode_torque(msg.data[4], msg.data[5])

            print(f"Fx:{Fx:7.2f}N  Fy:{Fy:7.2f}N  Fz:{Fz:7.2f}N  "
                  f"Tx:{Tx:6.3f}Nm Ty:{Ty:6.3f}Nm Tz:{Tz:6.3f}Nm")

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()