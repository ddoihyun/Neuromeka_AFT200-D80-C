# FT_control — 6축 F/T 센서 기반 Damping 어드미턴스 제어기

Nuri Robot(IndyDCP3)의 텔레오퍼레이션을 위한 **6축 Force/Torque 센서 기반 Damping 어드미턴스 제어기**입니다.  
CAN 버스를 통해 수신한 F/T 센서 데이터를 실시간으로 처리하여 로봇의 TCP 위치를 직접 제어합니다.

---

## 제어 원리

순수 Damping 모델을 사용하여 힘/토크 입력을 속도 명령으로 변환합니다.

```
D · ẋ = F
ẋ = F / D
```

변환된 속도는 6D 상대 태스크 포즈로 적분되어 `MoveTeleL`에 전달됩니다.

| 채널 | 단위 |
|------|------|
| 병진 (X, Y, Z) | mm |
| 회전 (Rx, Ry, Rz) | deg |

---

## 요구 사항

- Python 3.8 이상
- Ubuntu (테스트 환경: Ubuntu 20.04 / 22.04)
- CAN-USB 어댑터 (예: FT232 기반 slcan 장치)
- Nuri Robot (IndyDCP3 지원 모델)

### Python 패키지

```bash
pip install pyserial
pip install python-can
pip install neuromeka
```

---

## 설치 및 환경 설정

### 1. 프로젝트 디렉토리 이동

```bash
cd release/FT_control
```

### 2. 가상환경 생성 및 활성화

```bash
# 최초 1회만 실행
python3 -m venv venv

# 매번 실행 시 활성화
source venv/bin/activate
```

정상 활성화 시 터미널 프롬프트 앞에 `(venv)` 표시가 나타납니다.

```
(venv) user@STEP2:~/release/FT_control$
```

### 3. 패키지 설치

```bash
pip install pyserial python-can neuromeka
```

---

## 하드웨어 설정

> 자세한 내용은 [H/W Setup 가이드](https://www.notion.so/H-W-Setup-3668dce2e0b180cfb6cec0ec8a95e1ff?pvs=21)를 참고하세요.

### CAN 인터페이스 확인

USB 장치 연결 확인:

```bash
sudo lsusb
# 예시: Bus 001 Device 003: ID 0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC
```

포트 번호 확인:

```bash
ls /dev/ttyUSB*
# 예시: /dev/ttyUSB0
```

확인한 포트 번호를 `config.json`의 `can.channel` 항목에 입력합니다.

### 접근 권한 설정 (최초 1회)

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

---

## 설정 파일 (`config.json`)

| 섹션 | 항목 | 설명 |
|------|------|------|
| `can` | `interface` | CAN 인터페이스 종류 (기본값: `slcan`) |
| `can` | `channel` | USB 포트 경로 (예: `/dev/ttyUSB0`) |
| `can` | `bitrate` | CAN 통신 속도 (기본값: `1000000`) |
| `can` | `force_id` / `torque_id` | F/T 데이터 CAN 메시지 ID |
| `robot` | `ip` | 로봇 IP 주소 |
| `robot` | `apply_robot_commands` | `false`로 설정 시 로봇 명령 없이 센서만 테스트 (디버그 모드) |
| `robot` | `vel_ratio` / `acc_ratio` | 속도/가속도 비율 (0.0 ~ 1.0) |
| `control.common` | `period_sec` | 제어 루프 주기 (기본값: `0.01`초 = 100Hz) |
| `control.common` | `bias_sample_count` | 바이어스 측정 샘플 수 (기본값: `200`) |
| `control.common` | `force_threshold` | 동작 시작 힘 임계값 (N) |
| `control.common` | `force_release_threshold` | 동작 해제 힘 임계값 (N) |
| `control.damping` | `damping_force` | 병진 방향 기본 감쇠 계수 |
| `control.damping` | `damping_torque` | 회전 방향 기본 감쇠 계수 |
| `control.protection` | `enabled` | 특이점 보호 활성화 여부 (`true` / `false`) |
| `control.protection` | `cond_slow/strong/stop` | 조건수(Condition Number) 기반 위험 단계 임계값 |
| `control.protection` | `sigma_slow/strong/stop` | 최소 특이값(Singular Value) 기반 위험 단계 임계값 |
| `logging` | `log_level` | 로그 레벨 (`DEBUG`, `INFO`, `WARNING`, `ERROR`) |
| `logging` | `log_to_file` | 파일 로깅 여부 |
| `logging` | `log_output_dir` | 로그 파일 저장 디렉토리 |

---

## 실행

### F/T 센서 연결 확인 (선택)

```bash
sudo python3 sample.py
```

### 메인 제어 코드 실행 (Damping 버전)

```bash
sudo python3 main.py
```

커스텀 설정 파일 지정 시:

```bash
sudo python3 main.py -c /path/to/config.json
# 또는
sudo python3 main.py /path/to/config.json
```

실행 후 프롬프트가 나타나면 로봇과 F/T 센서를 **정지 상태로 유지**한 뒤 Enter를 눌러 바이어스 측정을 진행합니다.

```
Keep robot and F/T sensor still, then press Enter to measure bias...
```

종료는 `Ctrl+C`를 누르면 텔레오퍼레이션이 안전하게 종료됩니다.

---

## 특이점 보호 (Singularity Protection)

`config.json`의 `control.protection.enabled`가 `true`인 경우, 매 루프마다 로봇의 관절 각도로 MDH 기반 Jacobian을 계산하여 특이점 근접도를 판단합니다.

위험 단계에 따라 감쇠 계수가 자동으로 증가하며, 동작이 둔화됩니다.

| 단계 | 조건 | 동작 |
|------|------|------|
| `normal` | 특이점 미감지 | 기본 damping 적용 |
| `cond_slow` / `sigma_slow` | 특이점 접근 시작 | damping 소폭 증가 |
| `cond_strong` / `sigma_strong` | 특이점 근접 | damping 대폭 증가 |
| `cond_stop` / `sigma_stop` | 특이점 매우 근접 | 최대 damping 적용 |

---

## 로봇 운용 상태 (op_state)

제어 루프 중 아래 상태만 정상으로 허용됩니다 (`config.json`의 `op_state.allowed_states`로 설정):

| 코드 | 상태명 |
|------|--------|
| 17 | `TELE_OP` |
| 6 | `MOVING` |

허용되지 않은 상태로 전환되면 제어 루프가 즉시 종료됩니다.

---

## 로그

실행 시 `logs/` 디렉토리에 타임스탬프 기반 로그 파일이 생성됩니다.

```
logs/main_protect_20240604_153022.log
```

`log_level`을 `DEBUG`로 설정하면 매 루프마다 F/T 원시값, 보정값, 명령 속도, 포즈, 특이점 지표 등 상세 데이터가 기록됩니다.

---

## 프로젝트 구조

```
FT_control/
├── main.py          # 메인 제어 코드 (Damping 어드미턴스)
├── config.json      # 설정 파일
├── sample.py        # F/T 센서 연결 확인용 샘플
├── logs/            # 로그 파일 저장 디렉토리 (자동 생성)
└── venv/            # Python 가상환경
```

---

## 문제 해결

**CAN 장치를 찾을 수 없는 경우**
- `ls /dev/ttyUSB*`로 포트 확인 후 `config.json`의 `channel` 수정
- 권한 오류 시 `sudo usermod -aG dialout $USER` 실행 후 재로그인

**F/T 센서 데이터 타임아웃**
- 센서 전원 및 CAN 케이블 연결 상태 확인
- `config.json`의 `can.bitrate`가 센서 설정과 일치하는지 확인

**로봇 연결 실패**
- `config.json`의 `robot.ip`가 올바른지 확인
- 로봇과 PC가 동일 네트워크에 있는지 확인 (`ping 192.168.0.86`)
- 로봇의 op_state가 정상 범위(`IDLE`, `MOVING`)인지 확인

**디버그 모드 (로봇 없이 센서만 테스트)**  
`config.json`에서 `apply_robot_commands`를 `false`로 설정하면 로봇 연결 없이 F/T 센서 데이터만 확인할 수 있습니다.