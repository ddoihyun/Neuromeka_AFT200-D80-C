# FT Control 설명서

## 1. 이 프로젝트가 하는 일

이 프로젝트는 6축 F/T 센서 값을 읽어서 Indy 로봇의 TCP 상대 이동 명령을 만드는 제어 코드이다.

센서에서 들어온 힘/토크를 bias 보정한 뒤, 두 가지 방식 중 하나로 `command_pose`를 계산한다. `command_pose`는 최종적으로 Indy `MoveTeleL`에 들어가는 로봇 입력값이다.

```text
F/T sensor
  -> CAN receive
  -> bias compensation
  -> controller
  -> command_pose
  -> MoveTeleL
```

현재 제어 파일은 두 개다.

| 파일 | 제어 방식 | 핵심 특징 |
| --- | --- | --- |
| `tracking_rate_ver.py` | virtual target + tracking rate | 힘으로 `virtual_pose`를 만들고, `command_pose`가 이를 따라간다. |
| `damping_ver.py` | damping-only admittance | 힘을 바로 속도로 바꿔 `command_pose`를 적분한다. |

## 2. 파일 구조

```text
FT_control/
├── config.json
├── tracking_rate_ver.py
├── damping_ver.py
└── control.md
```

| 파일 | 역할 |
| --- | --- |
| `config.json` | CAN, 로봇, 공통 제어값, 방식별 제어값 관리 |
| `tracking_rate_ver.py` | 기존 virtual target 기반 추종 제어 |
| `damping_ver.py` | 힘을 속도로 해석하는 damping-only 제어 |
| `control.md` | 구조와 제어 원리 설명 |

## 3. 실행 방법

기본 설정 파일은 루트의 `config.json`이다.

```powershell
py .\tracking_rate_ver.py
py .\damping_ver.py
```

설정 파일을 명시하려면 `-c` 또는 `--config`를 사용한다.

```powershell
py .\tracking_rate_ver.py -c .\config.json
py .\damping_ver.py --config .\config.json
```

`robot.apply_robot_commands`가 `false`이면 로봇에는 명령을 보내지 않고 계산과 로그만 수행한다. 실제 로봇에 명령을 보내려면 `true`로 바꿔야 한다.

## 4. 설정 구조

`config.json`의 `control`은 공통 설정과 방식별 설정으로 나뉜다.

```json
{
  "control": {
    "common": {
      "period_sec": 0.01,
      "bias_sample_count": 200,
      "stale_sensor_timeout_sec": 0.2,
      "force_threshold": 1.0,
      "release_hold_sec": 0.05
    },
    "tracking_rate": {
      "tracking_rate": 3.0
    },
    "damping": {
      "damping": 1.0
    }
  }
}
```

### 4.1 공통 설정

| 설정 | 기본값 | 의미 |
| --- | ---: | --- |
| `control.common.period_sec` | `0.01` | 제어 루프 주기. 0.01초이므로 100 Hz |
| `control.common.bias_sample_count` | `200` | 시작 시 bias 평균에 사용할 샘플 수 |
| `control.common.stale_sensor_timeout_sec` | `0.2` | 이 시간보다 센서 데이터가 오래되면 제어 중지 |
| `control.common.force_threshold` | `1.0` | 병진 force 3축 기준 입력 감지 threshold |
| `control.common.release_hold_sec` | `0.05` | `tracking_rate_ver.py`에서 force 해제 후 잔여 추종 오차를 지우기까지 기다리는 시간 |

현재 torque threshold는 없다. 무입력 판정은 `Fx`, `Fy`, `Fz`만 사용한다.

```text
force_detected = max(abs(Fx), abs(Fy), abs(Fz)) > force_threshold
```

### 4.2 tracking_rate 방식 설정

| 설정 | 기본값 | 의미 |
| --- | ---: | --- |
| `control.tracking_rate.tracking_rate` | `3.0` | `command_pose`가 `virtual_pose`를 따라가는 속도 |

### 4.3 damping 방식 설정

| 설정 | 기본값 | 의미 |
| --- | ---: | --- |
| `control.damping.damping` | `1.0` | 힘/토크를 속도로 바꾸는 damping 값 |

## 5. 공통 데이터 흐름

두 제어 파일 모두 큰 흐름은 같다.

1. 설정 파일을 읽는다.
2. CAN bus를 연다.
3. F/T 센서 수신 스레드를 시작한다.
4. 첫 센서 데이터가 들어올 때까지 기다린다.
5. 사용자가 Enter를 누르면 정지 상태에서 bias를 측정한다.
6. 100 Hz 루프를 시작한다.
7. 매 루프마다 `ft_comp = ft_raw - bias`를 계산한다.
8. 각 방식에 따라 `command_pose`를 갱신한다.
9. 실제 명령 모드이면 `MoveTeleL(tpos=command_pose)`를 보낸다.
10. 종료 시 teleop과 CAN bus를 정리한다.

## 6. F/T 센서 값 처리

CAN force frame은 다음 식으로 변환한다.

```text
Fx = (d0 * 256 + d1) / 100 - 300
Fy = (d2 * 256 + d3) / 100 - 300
Fz = (d4 * 256 + d5) / 100 - 300
```

CAN torque frame은 다음 식으로 변환한다.

```text
Tx = (d0 * 256 + d1) / 500 - 50
Ty = (d2 * 256 + d3) / 500 - 50
Tz = (d4 * 256 + d5) / 500 - 50
```

시작 시 측정한 bias를 빼서 제어 입력을 만든다.

```text
ft_comp[i] = ft_raw[i] - bias[i]
```

현재 코드에는 low-pass filter, 축별 gain, saturation, 축별 deadband는 없다. 다만 병진 force threshold를 이용해 입력이 없는 상태를 판단한다.

## 7. tracking_rate_ver.py 동작 원리

이 방식에는 두 개의 pose가 있다.

| 변수 | 의미 |
| --- | --- |
| `virtual_pose` | F/T 입력으로 움직이는 가상 목표 자세 |
| `command_pose` | 실제 로봇에 보내는 상대 task pose |

### 7.1 virtual_pose 생성

보정된 F/T 값을 그대로 적분해서 `virtual_pose`를 만든다.

```text
virtual_step[i] = ft_comp[i] * dt
virtual_pose[i] += virtual_step[i]
```

예를 들어 `Fx = 10 N`, `dt = 0.01 s`이면:

```text
virtual_step_x = 10 * 0.01 = 0.1 mm
```

즉 10 N이 1초 동안 유지되면 `virtual_pose_x`는 약 10 mm 이동한다.

### 7.2 command_pose 추종

`command_pose`는 `virtual_pose`를 즉시 따라가지 않고, 오차의 일부만 따라간다.

```text
error[i] = virtual_pose[i] - command_pose[i]
command_step[i] = tracking_rate * error[i] * dt
command_pose[i] += command_step[i]
```

기본값은 다음과 같다.

```text
tracking_rate = 3.0
dt = 0.01
command_step = error * 0.03
```

즉 매 루프마다 오차의 3%를 따라간다. 이 구조 때문에 `virtual_pose`와 `command_pose`는 정상적으로 차이가 날 수 있다.

### 7.3 force가 threshold 이하일 때

입력 force가 threshold보다 작아지면 더 이상 `virtual_pose`를 적분하지 않는다.

```text
if force_detected:
    virtual_pose += ft_comp * dt
else:
    virtual_pose 유지
```

다만 threshold 근처에서 순간적으로 입력이 끊겼다고 바로 멈추면 조작감이 끊길 수 있다. 그래서 마지막 force 입력 이후 `release_hold_sec` 동안은 남아 있는 `virtual_pose - command_pose` 오차를 계속 따라간다.

```text
if force_recent:
    command_pose가 virtual_pose를 추종
else:
    virtual_pose = command_pose
    command_step = 0
```

이 처리가 중요한 이유는, tracking-rate 방식에서는 힘을 놓은 뒤에도 `virtual_pose`와 `command_pose` 사이에 잔여 오차가 남을 수 있기 때문이다. 그 오차를 방치하면 로봇이 입력이 없는데도 계속 움직일 수 있다.

정리하면:

```text
force 있음:
  virtual_pose 갱신
  command_pose가 virtual_pose 추종

force가 방금 사라짐:
  virtual_pose는 멈춤
  command_pose가 잠깐 남은 오차 추종

force 없음이 release_hold_sec 이상 지속:
  virtual_pose를 command_pose에 맞춤
  command_step = 0
```

## 8. damping_ver.py 동작 원리

이 방식은 `virtual_pose`가 없다. `command_pose` 하나만 사용한다.

```text
D * x_dot = F
x_dot = F / D
```

코드에서는 다음처럼 계산한다.

```text
command_velocity[i] = ft_comp[i] / damping
command_step[i] = command_velocity[i] * dt
command_pose[i] += command_step[i]
```

예를 들어 `damping = 1.0`, `Fx = 10 N`, `dt = 0.01 s`이면:

```text
command_velocity_x = 10 mm/s
command_step_x = 0.1 mm
```

### 8.1 force가 threshold 이하일 때

damping 방식은 남아 있는 `virtual_pose - command_pose` 오차가 없다. 따라서 force가 threshold 이하이면 즉시 속도를 0으로 만든다.

```text
if force_detected:
    command_velocity = ft_comp / damping
    command_step = command_velocity * dt
else:
    command_velocity = 0
    command_step = 0
```

힘을 빼면 `command_pose`가 마지막 위치에 그대로 유지된다. 원점으로 돌아가는 stiffness 항은 없다.

## 9. 두 방식 비교

| 항목 | tracking_rate_ver.py | damping_ver.py |
| --- | --- | --- |
| 내부 목표점 | `virtual_pose` 있음 | 없음 |
| 로봇 입력 | `command_pose` | `command_pose` |
| 기본 입력 해석 | 힘/토크를 가상 목표 위치 변화로 적분 | 힘/토크를 속도로 변환 |
| 핵심 식 | `command_dot = tracking_rate * (virtual - command)` | `command_dot = ft_comp / damping` |
| 힘을 뺐을 때 | `release_hold_sec` 뒤 잔여 오차 제거 | 즉시 속도 0 |
| 튜닝 값 | `tracking_rate` | `damping` |
| 조작감 | 가상 목표를 따라가는 완충감 | 힘을 속도로 바꾸는 직접 조작감 |

## 10. 튜닝 가이드

### force_threshold

작게 잡으면 작은 힘에도 민감하게 반응한다. 대신 센서 노이즈나 bias drift 때문에 로봇이 천천히 계속 움직일 수 있다.

크게 잡으면 정지 안정성은 좋아진다. 대신 작은 조작 입력이 무시될 수 있다.

현재 기본값:

```text
force_threshold = 1.0 N
```

### release_hold_sec

`tracking_rate_ver.py`에 주로 의미가 있다. force가 threshold 아래로 내려간 뒤, 얼마 동안 남은 추종 오차를 따라갈지 정한다.

짧게 잡으면 힘을 놓자마자 잘 멈춘다. 길게 잡으면 조금 더 자연스럽게 목표점까지 따라가지만, 무입력 상태에서 움직임이 남을 수 있다.

현재 기본값:

```text
release_hold_sec = 0.05 s
```

### tracking_rate

`tracking_rate_ver.py`에서만 사용한다.

```text
tracking_rate 큼  -> virtual_pose를 빠르게 추종, 반응 민감
tracking_rate 작음 -> 부드럽지만 지연 증가
```

### damping

`damping_ver.py`에서만 사용한다.

```text
damping 큼  -> 같은 힘에서 느리게 이동
damping 작음 -> 같은 힘에서 빠르게 이동
```

현재는 6축 공통 damping 하나를 사용한다. 병진축과 회전축의 단위가 다르므로 실제 조작감 튜닝에는 축별 damping이 더 자연스럽다.

## 11. 현재 주의할 점

### 11.1 torque threshold가 없음

무입력 판정은 현재 force 3축만 본다.

```text
Fx, Fy, Fz
```

따라서 순수 torque 입력만으로 회전 조작을 해야 하는 상황에서는 입력이 없다고 판단될 수 있다. 회전 조작을 torque 중심으로 쓸 계획이면 torque threshold 또는 별도 회전 mode가 필요하다.

### 11.2 축별 제한이 없음

현재는 `command_step`, `command_pose`, `command_velocity`, `virtual_pose`에 제한이 없다.

실제 로봇 적용 전에는 최소한 다음 제한을 두는 것이 좋다.

```text
max_velocity
max_step_per_cycle
max_relative_pose
max_rotation
```

### 11.3 force/torque 프레임 freshness가 완전하지 않음

현재 센서 freshness는 force frame 또는 torque frame 중 하나만 들어와도 갱신될 수 있다. 더 안전하게 하려면 force와 torque의 timestamp를 분리하고, 둘 다 최신일 때만 complete sample로 보는 것이 좋다.

### 11.4 실제 TCP feedback을 사용하지 않음

현재 코드는 마지막으로 보낸 `command_pose`를 로봇이 따라갔다고 가정한다. 실제 로봇이 제한이나 지연 때문에 따라가지 못해도 제어 루프는 이를 모른다.

가능하면 실제 TCP pose를 읽어서 다음을 감시해야 한다.

```text
abs(command_pose - measured_tcp_relative_pose)
```

## 12. 다음 개선 추천

1. force/torque frame별 timestamp를 분리한다.
2. `max_velocity`, `max_step`, `max_pose` 제한을 추가한다.
3. `damping_ver.py`에 축별 damping을 추가한다.
4. 회전 조작이 필요하면 torque threshold를 별도로 추가한다.
5. 실제 TCP feedback 기반 안전 감시를 추가한다.
6. 연속 control period overrun이 발생하면 fail-safe 정지한다.

