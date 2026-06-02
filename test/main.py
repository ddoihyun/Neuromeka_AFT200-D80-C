"""
AFT200-D80 + 시스템베이스 uCAN V3.0
6축 힘/토크 센서 데이터 수신
"""

import can
import time
import csv
from datetime import datetime

# ── 설정 ──────────────────────────────────────
COM_PORT    = 'COM3'
CAN_BRATE   = 1000000    # 1Mbps
SENSOR_ID   = 0x01       # 기본 CAN ID
SAVE_CSV    = True       # CSV 저장 여부
# ──────────────────────────────────────────────

def decode_force(hi, lo):
    return (hi * 256 + lo) / 100.0 - 300.0

def decode_torque(hi, lo):
    return (hi * 256 + lo) / 500.0 - 50.0

def main():
    bus = can.Bus(
        interface='slcan',
        channel=COM_PORT,
        bitrate=CAN_BRATE,
        sleep_after_open=2.0,
        rtscts=False
    )
    print("✅ Connected to AFT200-D80\n")

    # 연속 전송 시작 명령
    bus.send(can.Message(
        arbitration_id=0x102,
        data=[SENSOR_ID, 0x03, 0x01],
        is_extended_id=False
    ))
    time.sleep(0.1)

    Fx = Fy = Fz = Tx = Ty = Tz = 0.0

    # CSV 파일 준비
    csvfile = None
    writer  = None
    if SAVE_CSV:
        fname = datetime.now().strftime("aft200_%Y%m%d_%H%M%S.csv")
        csvfile = open(fname, 'w', newline='')
        writer  = csv.writer(csvfile)
        writer.writerow(['timestamp', 'Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz'])
        print(f"📄 Saving to {fname}\n")

    print("Press Ctrl+C to stop\n")
    print(f"{'Fx':>10} {'Fy':>10} {'Fz':>10}  |  "
          f"{'Tx':>10} {'Ty':>10} {'Tz':>10}")
    print("-" * 75)

    try:
        while True:
            msg = bus.recv(timeout=2.0)
            if msg is None:
                print("[WARN] Timeout")
                continue

            d = msg.data
            updated = False

            if msg.arbitration_id == SENSOR_ID:          # 힘 프레임
                Fx = decode_force(d[0], d[1])
                Fy = decode_force(d[2], d[3])
                Fz = decode_force(d[4], d[5])
                updated = True

            elif msg.arbitration_id == SENSOR_ID + 1:    # 토크 프레임
                Tx = decode_torque(d[0], d[1])
                Ty = decode_torque(d[2], d[3])
                Tz = decode_torque(d[4], d[5])
                updated = True

            if updated:
                ts = time.time()
                print(f"{Fx:>10.3f}N {Fy:>10.3f}N {Fz:>10.3f}N  |  "
                      f"{Tx:>9.4f}Nm {Ty:>9.4f}Nm {Tz:>9.4f}Nm")
                if writer:
                    writer.writerow([f"{ts:.4f}",
                                     f"{Fx:.4f}", f"{Fy:.4f}", f"{Fz:.4f}",
                                     f"{Tx:.4f}", f"{Ty:.4f}", f"{Tz:.4f}"])

    except KeyboardInterrupt:
        print("\n✅ Stopped.")
    finally:
        if csvfile:
            csvfile.close()
            print(f"💾 Data saved.")
        bus.shutdown()

if __name__ == "__main__":
    main()