"""
uCAN slcan 수동 초기화 테스트
"""
import serial
import time

COM_PORT = 'COM3'
SENSOR_ID = 0x01

ser = serial.Serial(COM_PORT, 115200, timeout=1.0)
time.sleep(0.5)

# slcan 프로토콜: CAN 속도 설정 및 열기
# S8 = 1Mbps, S6 = 500kbps, S5 = 250kbps
ser.write(b'\r')          # 이전 명령 초기화
time.sleep(0.1)
ser.write(b'S8\r')        # CAN 속도 1Mbps 설정
time.sleep(0.1)
ser.write(b'O\r')         # CAN 버스 열기
time.sleep(0.2)

print("Bus opened. Sending start command...")

# AFT200 연속 전송 명령: ID=0x102, DLC=3, data=01 03 01
# slcan STD 프레임 포맷: t[ID 3자리][DLC][DATA...]\r
cmd = b't10230103 01\r'   # ← 아래에서 포맷 설명 참고
# 올바른 포맷:
cmd = bytes('t{:03X}{:01X}{}\r'.format(
    0x102, 3, '010301'
), 'ascii')
print(f"Sending: {cmd}")
ser.write(cmd)

time.sleep(0.2)

# 응답 확인
resp = ser.read(32)
print(f"Response: {resp}")

print("\nListening for CAN frames (10 seconds)...")
ser.timeout = 0.1
start = time.time()
while time.time() - start < 10:
    line = ser.read_until(b'\r')
    if line:
        print(f"RAW: {line}")

ser.write(b'C\r')  # CAN 버스 닫기
ser.close()
print("Done.")