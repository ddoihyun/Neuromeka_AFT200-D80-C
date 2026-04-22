#***********************************************************************
#
# Author: AIDIN ROBOTICS <info@aidinrobotics.com>
# Date: 12-08-2023
#
# 이 코드는 AFT 6축 힘/토크 센서에서 데이터를 받아오는 예제
#
#***********************************************************************/

import struct   # 바이너리 데이터 변환용 (현재 코드에서는 거의 안 쓰임)
import socket   # TCP 통신용 라이브러리

# 센서의 IP 주소 (센서와 같은 네트워크에 있어야 함)
IP_ADDR = '192.168.0.223'

# 센서가 사용하는 TCP 포트
PORT = 4001

# ===== 센서 명령 관련 상수 =====

# 센서 ID (여러 센서를 구분할 때 사용 가능)
CMD_TYPE_SENSOR_ID = '01'

# 데이터 전송 모드 설정
SENSOR_TRANSMIT_TYPE_MODE = '03'

# 온도 보정 활성화 설정
SENSOR_TRANSMIT_TYPE_SET_TEMP = '01'

# TCP 소켓 생성 (IPv4, TCP 방식)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def main():
    global s

    # 소켓 타임아웃 설정 (2초 안에 응답 없으면 에러)
    s.settimeout(2.0)

    # 센서에 TCP 연결
    s.connect((IP_ADDR, PORT))

    # ===== 센서 초기 명령 전송 =====
    # 센서에게 "데이터 스트리밍 시작" 요청하는 패킷 구성

    # 패킷 구조 (hex string 이어붙임)
    sendData = (
        '04' +  # 헤더
        '00' + '00' + '01' +  # 길이/명령 관련 필드
        '02' +                # 명령 타입
        '06' +                # 데이터 길이
        CMD_TYPE_SENSOR_ID +  # 센서 ID
        SENSOR_TRANSMIT_TYPE_MODE +  # 전송 모드
        SENSOR_TRANSMIT_TYPE_SET_TEMP  # 온도 보정 활성화
    )

    # hex 문자열 → 실제 바이트 배열로 변환
    sendData = bytearray.fromhex(sendData)

    # 센서로 명령 전송
    s.send(sendData)

    # 초기 응답 수신 (보통 ACK)
    recvData = recvMsg()

    # ===== 변수 초기화 =====
    # 힘 (Force)
    Fx = 0
    Fy = 0
    Fz = 0

    # 토크 (Torque)
    Tx = 0
    Ty = 0
    Tz = 0

    # ===== 데이터 수신 루프 =====
    # 10000번 반복해서 데이터 읽기
    for i in range(10000):

        # 센서에서 14바이트 데이터 수신
        recvData = recvMsg()

        # recvData[4]는 데이터 타입 구분용
        # 0x01 → 힘 데이터 (Fx, Fy, Fz)
        # 0x02 → 토크 데이터 (Tx, Ty, Tz)

        if recvData[4] == 0x01:
            # ===== 힘 데이터 처리 =====

            # 16비트 데이터 조합 (상위바이트 *256 + 하위바이트)
            # → 실제 값으로 변환 (스케일 + offset 적용)
            Fx = ((recvData[6]*256 + recvData[7]) / 100.0) - 300.0
            Fy = ((recvData[8]*256 + recvData[9]) / 100.0) - 300.0
            Fz = ((recvData[10]*256 + recvData[11]) / 100.0) - 300.0

            # print("Fx : " + str(round(Fx,2)) + ...)

        elif recvData[4] == 0x02:
            # ===== 토크 데이터 처리 =====

            Tx = ((recvData[6]*256 + recvData[7]) / 500.0) - 50.0
            Ty = ((recvData[8]*256 + recvData[9]) / 500.0) - 50.0
            Tz = ((recvData[10]*256 + recvData[11]) / 500.0) - 50.0

            # print("Tx : " + str(round(Tx,2)) + ...)

        # ===== 결과 출력 =====
        # 힘 + 토크를 한 줄로 출력
        print(
            "Fx : " + str(round(Fx, 2)) + " " +
            "Fy : " + str(round(Fy, 2)) + " " +
            "Fz : " + str(round(Fz, 2)) + " " +
            "Tx : " + str(round(Tx, 2)) + " " +
            "Ty : " + str(round(Ty, 2)) + " " +
            "Tz : " + str(round(Tz, 2)) + " "
        )

    # 소켓 연결 종료
    s.close()


def recvMsg():
    # ===== 데이터 수신 함수 =====

    # 센서에서 14바이트 읽기
    recvData = bytearray(s.recv(14))

    # 디버그용 출력
    printMsg(recvData)

    return recvData


def printMsg(msg):
    # ===== raw 데이터 출력 함수 =====

    dataStr = "DATA: "

    # 실제 센서 데이터 부분 (6~13 바이트)
    for i in range(6, 14):
        dataStr += str(msg[i]) + " "

    # print(dataStr)  # 필요할 때만 활성화


# 프로그램 시작점
if __name__ == "__main__":
    main()