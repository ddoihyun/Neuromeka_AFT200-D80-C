# 게인 테스트 및 축별 안전 튜닝 가이드

## 1. 현재 목표

현재 제어는 직접교시 느낌을 목표로 한다.

원하는 감각은 다음과 같다.

```text
힘/토크를 주면 가상 포인트가 움직임
로봇 명령 포즈는 그 가상 포인트를 부드럽게 따라감
힘을 놓으면 급격히 튀지 않고 안정적으로 멈춤
```

지금 느낌이 "빡빡하다"는 것은 보통 다음 중 하나다.

- 같은 힘을 줘도 가상 포인트가 충분히 이동하지 않는다.
- 가상 포인트는 움직이지만 로봇 명령 포즈가 너무 천천히 따라온다.
- deadband가 커서 초반 입력이 잘 안 먹는다.
- 안전 제한이 낮아서 계산상 step이 자주 잘린다.

한 번에 크게 빠르게 만드는 것은 위험하므로, 아래 순서대로 하나씩 조정한다.

## 2. 축별 테스트 모드

코드에 다음 변수가 추가되어 있다.

```python
AXIS_TEST_MODE = 'X'
```

가능한 값은 다음과 같다.

```text
'X'   : Fx와 Tx만 사용
'Y'   : Fy와 Ty만 사용
'Z'   : Fz와 Tz만 사용
'ALL' : Fx, Fy, Fz, Tx, Ty, Tz 전체 사용
```

예를 들어 `AXIS_TEST_MODE = 'X'`이면 다음만 활성화된다.

```text
Fx -> X(tool) 이동
Tx -> Rx(tool) 회전
```

나머지 `Fy, Fz, Ty, Tz`는 bias/deadband 계산은 되지만 제어 입력 `wrench_eff`에서는 0으로 막힌다. 따라서 테스트 중 의도하지 않은 다른 축 이동을 줄일 수 있다.

권장 테스트 순서:

```text
1. APPLY_ROBOT_COMMANDS = False
2. AXIS_TEST_MODE = 'X'
3. 로그에서 방향과 크기 확인
4. AXIS_TEST_MODE = 'Y'
5. AXIS_TEST_MODE = 'Z'
6. 모든 축 방향이 맞으면 AXIS_TEST_MODE = 'ALL'
7. 마지막에만 APPLY_ROBOT_COMMANDS = True
```

## 3. 디버깅 모드와 실제 적용 모드

기본은 디버깅 모드로 둔다.

```python
APPLY_ROBOT_COMMANDS = False
```

이 모드에서는 실제 `MoveTeleL` 명령이 나가지 않는다. F/T 센서 입력, 가상 포인트, 계산된 명령 방향만 로그로 확인한다.

실제 로봇에 적용할 때만 다음처럼 바꾼다.

```python
APPLY_ROBOT_COMMANDS = True
```

축별 로그가 예상과 다르게 나오면 실제 적용 모드로 넘어가지 않는다.

## 4. 튜닝 파라미터 역할

### 4.1 가상 포인트 민감도

```python
VIRTUAL_POINT_FORCE_GAIN_X
VIRTUAL_POINT_FORCE_GAIN_Y
VIRTUAL_POINT_FORCE_GAIN_Z
VIRTUAL_POINT_TORQUE_GAIN_RX
VIRTUAL_POINT_TORQUE_GAIN_RY
VIRTUAL_POINT_TORQUE_GAIN_RZ
```

이 값은 힘/토크가 가상 포인트를 얼마나 빨리 미는지를 정한다.

값을 키우면:

```text
같은 힘에도 virtual_pose가 더 빨리 이동
사용자 입장에서는 덜 빡빡함
너무 키우면 가상 포인트가 멀리 도망가서 로봇이 급히 따라갈 수 있음
```

값을 낮추면:

```text
같은 힘에도 virtual_pose가 천천히 이동
안전하지만 둔하고 빡빡하게 느껴짐
```

### 4.2 Stiffness / Damping

```python
STIFFNESS_X
STIFFNESS_Y
STIFFNESS_Z
DAMPING_X
DAMPING_Y
DAMPING_Z

ROT_STIFFNESS_RX
ROT_STIFFNESS_RY
ROT_STIFFNESS_RZ
ROT_DAMPING_RX
ROT_DAMPING_RY
ROT_DAMPING_RZ
```

실제 추종 속도는 주로 `K / D` 비율로 결정된다.

```text
K / D 증가 -> 로봇 명령 포즈가 가상 포인트를 더 빨리 따라감
K / D 감소 -> 더 느리고 부드럽게 따라감
```

중요한 점:

```text
가상 포인트 민감도 = 사람이 미는 손잡이가 얼마나 잘 움직이는가
K / D = 로봇이 그 손잡이를 얼마나 빨리 따라가는가
```

둘을 섞어서 한 번에 바꾸면 원인 파악이 어려워진다.

## 5. "빡빡함"을 줄이는 점진적 조정 순서

### Step 1. 실제 제어 전 디버깅 로그 확인

먼저 다음 상태로 둔다.

```python
APPLY_ROBOT_COMMANDS = False
AXIS_TEST_MODE = 'X'
```

X 방향으로 힘을 줬을 때 로그에서 다음을 본다.

```text
EFF의 Fx가 기대한 부호로 나오는가
virtual_step의 X가 기대한 방향으로 증가하는가
cmd_step의 X가 같은 방향으로 나오는가
다른 축 cmd_step은 0인가
```

이 단계에서 방향이 틀리면 gain을 조정하지 말고 축 부호/좌표계부터 수정한다.

### Step 2. Deadband가 너무 큰지 확인

힘을 조금 줬는데 `COMP F`는 변하지만 `EFF`가 계속 0이면 threshold가 너무 클 수 있다.

조정 후보:

```python
FORCE_THRESHOLD_X
FORCE_THRESHOLD_Y
FORCE_THRESHOLD_Z
TORQUE_THRESHOLD_RX
TORQUE_THRESHOLD_RY
TORQUE_THRESHOLD_RZ
```

권장:

```text
한 번에 20% 이하로만 낮춘다.
노이즈로 EFF가 흔들리기 시작하면 다시 올린다.
```

예:

```python
FORCE_THRESHOLD_X = 0.5 -> 0.4
TORQUE_THRESHOLD_RX = 0.05 -> 0.04
```

### Step 3. 가상 포인트 민감도를 먼저 올린다

방향은 맞는데 사람이 미는 느낌이 너무 빡빡하면 먼저 가상 포인트 gain을 조금 올린다.

X/Y/Z 방향:

```python
VIRTUAL_POINT_FORCE_GAIN_X = 2.0
```

권장 증가:

```text
2.0 -> 2.4 -> 3.0
```

회전:

```python
VIRTUAL_POINT_TORQUE_GAIN_RX = 5.0
```

권장 증가:

```text
5.0 -> 6.0 -> 7.5
```

이 단계에서는 `K/D`는 그대로 둔다.

### Step 4. 로봇 추종이 답답하면 K/D를 조정한다

가상 포인트는 잘 움직이는데 `command_pose`가 너무 늦게 따라오면 `K/D`를 키운다.

방법 1: stiffness를 올린다.

```python
STIFFNESS_X = 1.0 -> 1.2 -> 1.5
```

방법 2: damping을 낮춘다.

```python
DAMPING_X = 0.25 -> 0.22 -> 0.20
```

둘 중 하나만 선택해서 조정한다. 처음에는 damping을 낮추기보다 stiffness를 조금 올리는 쪽이 직관적이다.

주의:

```text
K/D를 크게 올리면 로봇이 가상 포인트를 급하게 따라가므로 실제 적용 전 반드시 DEBUG_ONLY에서 cmd_step 크기를 확인한다.
```

### Step 5. 실제 적용 전 command step 제한을 보수적으로 둔다

실제 적용 시에는 아래 값이 마지막 안전장치다.

```python
MAX_COMMAND_STEP_MM = 2.0
MAX_COMMAND_STEP_DEG = 0.25
```

처음 실제 적용할 때 더 안전하게 시작하려면:

```python
MAX_COMMAND_STEP_MM = 0.5
MAX_COMMAND_STEP_DEG = 0.05
```

방향과 반응이 확인되면 조금씩 올린다.

```text
MAX_COMMAND_STEP_MM: 0.5 -> 0.8 -> 1.0 -> 1.5
MAX_COMMAND_STEP_DEG: 0.05 -> 0.08 -> 0.10 -> 0.15
```

## 6. 축별 테스트 체크리스트

### X축 테스트

설정:

```python
AXIS_TEST_MODE = 'X'
```

확인:

```text
Fx를 주면 X(tool) 방향 virtual_step 발생
Tx를 주면 Rx(tool) 방향 virtual_step 발생
Y, Z, Ry, Rz 명령은 0에 가까움
```

### Y축 테스트

설정:

```python
AXIS_TEST_MODE = 'Y'
```

확인:

```text
Fy를 주면 Y(tool) 방향 virtual_step 발생
Ty를 주면 Ry(tool) 방향 virtual_step 발생
X, Z, Rx, Rz 명령은 0에 가까움
```

### Z축 테스트

설정:

```python
AXIS_TEST_MODE = 'Z'
```

확인:

```text
Fz를 주면 Z(tool) 방향 virtual_step 발생
Tz를 주면 Rz(tool) 방향 virtual_step 발생
X, Y, Rx, Ry 명령은 0에 가까움
```

### 전체 축 테스트

설정:

```python
AXIS_TEST_MODE = 'ALL'
```

확인:

```text
각 힘/토크 입력이 기대한 tool 축 방향으로 반응
여러 축이 동시에 움직여도 불안정한 누적이 없음
힘을 놓으면 cmd_step이 점진적으로 작아짐
```

## 7. 추천 초기 테스트 절차

1. `APPLY_ROBOT_COMMANDS = False`
2. `AXIS_TEST_MODE = 'X'`
3. 손으로 X 방향 힘과 X축 회전 토크를 작게 가함
4. 로그의 `EFF`, `virtual_step`, `cmd_step`, `axes` 확인
5. X 방향이 맞으면 `AXIS_TEST_MODE = 'Y'`
6. Y 방향 확인 후 `AXIS_TEST_MODE = 'Z'`
7. 세 축 모두 방향이 맞으면 gain을 10~20% 단위로 조정
8. 마지막에 `MAX_COMMAND_STEP_*`을 낮춘 상태로 `APPLY_ROBOT_COMMANDS = True`
9. 실제 적용 후 반응이 너무 느리면 가상 포인트 gain부터 조금씩 올림

## 8. 추천 우선순위

빡빡함을 줄일 때 추천 순서는 다음이다.

```text
1. deadband가 너무 큰지 확인
2. virtual point gain을 10~20%씩 증가
3. 그래도 추종이 늦으면 K/D를 10~20%씩 증가
4. 실제 적용 전 MAX_COMMAND_STEP은 낮게 시작
5. 축별 테스트가 끝나기 전에는 ALL 모드로 가지 않음
```

한 번에 여러 파라미터를 바꾸면 어떤 변화가 효과를 냈는지 알기 어렵다. 한 번에 하나만 바꾸고, 로그에서 `virtual_step`과 `cmd_step`을 나눠서 보는 것이 가장 안전하다.
