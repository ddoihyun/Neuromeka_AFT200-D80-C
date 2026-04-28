from neuromeka import IndyDCP3, JointTeleopType, TaskTeleopType
import time

# 로봇 연결
indy = IndyDCP3(robot_ip='192.168.0.161', index=0)


# ─────────────────────────────────────────────
# 1. 텔레오퍼레이션 상태 및 장치 정보 확인
# ─────────────────────────────────────────────
print("=== 텔레오퍼레이션 상태 확인 ===")
teleop_state = indy.get_teleop_state()
print("현재 텔레오퍼레이션 상태:", teleop_state)

teleop_device_info = indy.get_teleop_device()
print("연결된 텔레오퍼레이션 장치:", teleop_device_info)


# ─────────────────────────────────────────────
# 2. 관절 공간 텔레오퍼레이션 (상대좌표)
# ─────────────────────────────────────────────
print("\n=== 관절 공간 텔레오퍼레이션 (RELATIVE) ===")

indy.start_teleop(method=JointTeleopType.RELATIVE)
print("텔레오퍼레이션 시작 (Joint Relative)")

# 1번 관절을 현재 위치 기준 +10도 이동
indy.movetelej_rel(jpos=[10, 0, 0, 0, 0, 0], vel_ratio=0.5, acc_ratio=1.0)
print("관절 1 → +10도 이동")
time.sleep(2)

# 1번 관절을 원래 위치로 복귀 (-10도)
indy.movetelej_rel(jpos=[-10, 0, 0, 0, 0, 0], vel_ratio=0.5, acc_ratio=1.0)
print("관절 1 → -10도 복귀")
time.sleep(2)

indy.stop_teleop()
time.sleep(0.5)
print("텔레오퍼레이션 종료")


# ─────────────────────────────────────────────
# 3. 작업 공간 텔레오퍼레이션 (상대좌표)
# ─────────────────────────────────────────────
print("\n=== 작업 공간 텔레오퍼레이션 (RELATIVE) ===")

indy.start_teleop(method=TaskTeleopType.RELATIVE)
time.sleep(1)
print("텔레오퍼레이션 시작 (Task Relative)")

# X 방향으로 +50mm 이동
indy.movetelel_rel(tpos=[50, 0, 0, 0, 0, 0], vel_ratio=0.3, acc_ratio=1.0)
print("X 방향 → +50mm 이동")
time.sleep(2)

# Z 방향으로 +30mm 이동
indy.movetelel_rel(tpos=[0, 0, 30, 0, 0, 0], vel_ratio=0.3, acc_ratio=1.0)
print("Z 방향 → +30mm 이동")
time.sleep(2)

# 원래 위치로 복귀
indy.movetelel_rel(tpos=[-50, 0, -30, 0, 0, 0], vel_ratio=0.3, acc_ratio=1.0)
print("원위치 복귀")
time.sleep(2)

indy.stop_teleop()
time.sleep(0.5)
print("텔레오퍼레이션 종료")


# ─────────────────────────────────────────────
# 4. 관절 공간 텔레오퍼레이션 (절대좌표)
# ─────────────────────────────────────────────
print("\n=== 관절 공간 텔레오퍼레이션 (ABSOLUTE) ===")

indy.start_teleop(method=JointTeleopType.ABSOLUTE)
time.sleep(1) 
print("텔레오퍼레이션 시작 (Joint Absolute)")

# 모든 관절을 0도 위치(홈포지션)로 이동
indy.movetelej_abs(jpos=[0, 0, 0, 0, 0, 0], vel_ratio=0.5, acc_ratio=1.0)
print("홈 포지션으로 이동")
time.sleep(2)

indy.stop_teleop()
time.sleep(0.5)
print("텔레오퍼레이션 종료")


# ─────────────────────────────────────────────
# 5. 텔레오퍼레이션 모션 저장 / 불러오기 / 삭제
# ─────────────────────────────────────────────
print("\n=== 텔레오퍼레이션 모션 파일 관리 ===")

# 저장된 모션 파일 목록 확인
tele_file_list = indy.get_tele_file_list()
print("저장된 모션 파일 목록:", tele_file_list)

# 현재 모션 저장
indy.save_tele_motion(name="TestMotion1")
print("모션 저장 완료: TestMotion1")

# 저장된 모션 불러오기
indy.load_tele_motion(name="TestMotion1")
print("모션 불러오기 완료: TestMotion1")

# 모션 재생 속도 설정 및 확인
indy.set_play_rate(rate=1.5)
play_rate = indy.get_play_rate()
print("현재 재생 속도:", play_rate)

# 저장된 모션 삭제
indy.delete_tele_motion(name="TestMotion1")
print("모션 삭제 완료: TestMotion1")


# ─────────────────────────────────────────────
# 6. 텔레오퍼레이션 장치 연결 / 입력 읽기 / 해제
# ─────────────────────────────────────────────
# print("\n=== 텔레오퍼레이션 장치 연결 ===")

# try:
#     import neuromeka.proto.control_msgs as control_msgs

#     indy.connect_teleop_device(
#         name="TeleOpDevice1",
#         type=control_msgs.TeleOpDevice.JOYSTICK,
#         ip="192.168.0.2",
#         port=5555
#     )
#     print("텔레오퍼레이션 장치 연결 완료")

#     # 텔레오퍼레이션 키 입력 활성화
#     indy.enable_tele_key(enable=True)
#     print("텔레오퍼레이션 키 입력 활성화")

#     # 장치 입력값 읽기
#     teleop_input = indy.read_teleop_input()
#     print("현재 텔레오퍼레이션 입력값:", teleop_input)

#     # 텔레오퍼레이션 키 입력 비활성화
#     indy.enable_tele_key(enable=False)
#     print("텔레오퍼레이션 키 입력 비활성화")

#     # 장치 연결 해제
#     indy.disconnect_teleop_device()
#     print("텔레오퍼레이션 장치 연결 해제 완료")

# except Exception as e:
#     print("장치 연결 실패 (장치 미연결 환경):", e)

# print("\n=== 모든 예제 완료 ===")