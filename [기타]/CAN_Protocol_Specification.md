# CAN 통신 프로토콜 명세서 (FT 센서)

이 문서는 CAN 통신 기반 힘/토크(F/T) 센서의 제어 및 데이터 출력 프로토콜 명세입니다. 노션 페이지 또는 프로젝트 `README.md`로 즉시 사용할 수 있도록 구조화되어 있습니다.

---

## 1. 기본 설정 (Default Values)
초기 출고 시 센서의 기본 설정 값은 다음과 같습니다.

| 항목 | 설정 값 (Default) | 비고 |
| :--- | :--- | :--- |
| **Receiver ID** | `0x01` | 기본 CAN ID |
| **Data Output Rate** | `100Hz` | 기본 데이터 출력 속도 |

* 출력 속도는 `100Hz`에서 `1,000Hz`까지 설정 가능하며, 변경 방법은 아래 **[3.5 샘플 속도 설정 모드]**를 참조하십시오.

---

## 2. 모드 세팅 인덱스 개요 (Object Index)
센서의 설정 및 동작 모드를 변경하기 위해 사용하는 CAN 프레임의 인덱스 구조(`Table 1.4`) 요약입니다.
* **기본 Index:** `0x102`

| Index | Data field [0] | Data field [1] | Data field [2] | 설명 (Description) |
| :--- | :--- | :--- | :--- | :--- |
| **0x102** | `ID` | `0x00` | - | Default (기본 상태) |
| | `ID` | `0x01` | - | CAN ID 세팅 모드 |
| | `ID` | `0x02` | `0x00` | Bias set (온도 보상 미적용) |
| | `ID` | `0x02` | `0x01` | Bias set (온도 보상 적용) |
| | `ID` | `0x03` | `0x00` | 연속 전송 모드 (온도 보상 미적용) |
| | `ID` | `0x03` | `0x01` | 연속 전송 모드 (온도 보상 적용) |
| | `ID` | `0x05` | - | 샘플 속도(Sample rate) 설정 모드 |
| | `0xFF` | `0x00` | - | ID 확인 모드 |
| | `0xFF` | `0x0FF` | - | 팩토리 리셋 모드 (Factory Reset) |

---

## 3. 상세 명령 모드 및 제어 프로토콜

### 3.1 CAN ID 세팅 모드
센서의 CAN ID를 사용자가 원하는 값으로 변경하는 모드입니다.

#### Command Setting (명령 송신)
* 수신측(센서)은 설정된 ID로 변경을 반영합니다.

| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x102` | **Current ID** | `0x01` | **Setting ID** | `xx` | `xx` | `xx` | `xx` | `xx` |

* **Data field [0]:** 현재 설정되어 있는 센서의 ID (공장 기본값 `0x01`)
* **Data field [2]:** 새롭게 전송하여 변경할 사용자 설정 ID

#### Transmit (센서 응답)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **ID** | Setting ID | mode | `xx` | `xx` | `xx` | `xx` | `xx` | `xx` |

---

### 3.2 바이어스(Bias) 설정 모드
센서의 현재 측정값을 영점(0)으로 잡는 바이어스 타겟팅 설정입니다.

#### Command Setting (명령 송신)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x102` | **ID** | `0x02` | `0x00` 또는 `0x01` | `xx` | `xx` | `xx` | `xx` | `xx` |

* **Data field [2] = `0x00`:** 온도 보상 없이 바이어스 설정
  * *주의:* 이 명령은 온도 보상 없는 전송 모드 명령(`0x03` - `0x00`)과 함께 사용해야 합니다.
* **Data field [2] = `0x01`:** 온도 보상과 함께 바이어스 설정
  * *주의:* 이 명령은 온도 보상 있는 전송 모드 명령(`0x03` - `0x01`)과 함께 사용해야 합니다.

---

### 3.3 연속 전송 모드 (수신 데이터 활성화)
센서가 실시간으로 힘/토크 데이터를 연속으로 출력하도록 요청하는 명령입니다.

#### Command Setting (명령 송신)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x102` | **ID** | `0x03` | `0x00` 또는 `0x01` | `xx` | `xx` | `xx` | `xx` | `xx` |

* **Data field [2] = `0x00`:** 온도 보상 없이 전송 모드 활성화 (온도 보상 없는 바이어스 명령과 쌍으로 사용)
* **Data field [2] = `0x01`:** 온도 보상과 함께 전송 모드 활성화 (온도 보상 있는 바이어스 명령과 쌍으로 사용)

---

### 3.4 샘플 속도 (Sample Rate) 설정 모드
데이터 출력 주기를 결정하는 샘플링 주파수를 변경합니다.

#### Command Setting (명령 송신)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x102` | **ID** | `0x05` | Parameter [0] | Parameter [1] | `xx` | `xx` | `xx` | `xx` |

* **Data field [2]:** Sample rate parameter [0] (High Byte)
* **Data field [3]:** Sample rate parameter [1] (Low Byte)

#### 수식 및 계산법
* **샘플 파라미터 합산 공식:** $$\text{Sample rate parameter} = (\text{Parameter}[0] \times 256) + \text{Parameter}[1]$$
* **샘플 속도 결정 공식:**
  $$\text{Sample rate (Hz)} = \frac{1,000,000}{\text{Sample rate parameter}}$$

#### 주요 설정 값 예시 (Preset)
* **1000Hz:** Parameter[0] = `0x03`, Parameter[1] = `0xE8` (값: 1000)
* **500Hz:** Parameter[0] = `0x07`, Parameter[1] = `0xD0` (값: 2000)
* **333Hz:** Parameter[0] = `0x0B`, Parameter[1] = `0xB8` (값: 3003)
* **200Hz:** Parameter[0] = `0x13`, Parameter[1] = `0x88` (값: 5000)
* **100Hz:** Parameter[0] = `0x27`, Parameter[1] = `0x10` (값: 10000)

---

### 3.5 ID 확인 모드
현재 연결된 센서의 ID 정보를 질의하고 확인할 때 사용합니다.

#### Command Setting (명령 송신)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x102` | `0xFF` | `0x00` | `xx` | `xx` | `xx` | `xx` | `xx` | `xx` |

#### Transmit (센서 응답)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **ID** | `xx` | `xx` | `xx` | `xx` | `xx` | `xx` | `xx` | `xx` |

---

### 3.6 팩토리 리셋 모드 (Factory Reset)
센서의 모든 설정을 공장 출고 상태(초기화)로 되돌립니다.

#### Command Setting (명령 송신)
| Index | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `0x102` | `0xFF` | `0xFF` | `xx` | `xx` | `xx` | `xx` | `xx` | `xx` |

---

## 4. 데이터 출력 포맷 및 물리량 변환 (Data Output Format)
연속 전송 모드가 활성화되면 센서는 힘(Force)과 토크(Torque) 데이터를 각각 분리된 CAN ID 프레임으로 나누어 연속 전송합니다.

### 4.1 힘(Force) 데이터 구조 (`Table 1.9`)
* **CAN ID:** `ID` (센서 고유 ID)

| ID | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **ID** | Force Out [0] | Force Out [1] | Force Out [2] | Force Out [3] | Force Out [4] | Force Out [5] | `xx` | `xx` |

#### 1) RAW 데이터 합성 공식
* **Fx 출력 Raw:** $(\text{Force Output}[0] \times 256) + \text{Force Output}[1]$
* **Fy 출력 Raw:** $(\text{Force Output}[2] \times 256) + \text{Force Output}[3]$
* **Fz 출력 Raw:** $(\text{Force Output}[4] \times 256) + \text{Force Output}[5]$

#### 2) 물리량 변환 공식 (단위: Newtons [N])
$$\text{힘 [N]} = \frac{\text{Force Output Raw}}{100} - 300$$

---

### 4.2 토크(Torque) 데이터 구조 (`Table 1.10`)
* **CAN ID:** `ID + 1` (센서 고유 ID에서 1 증가된 값)

| ID | Data [0] | Data [1] | Data [2] | Data [3] | Data [4] | Data [5] | Data [6] | Data [7] |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **ID + 1** | Torque Out [0] | Torque Out [1] | Torque Out [2] | Torque Out [3] | Torque Out [4] | Torque Out [5] | `xx` | `xx` |

#### 1) RAW 데이터 합성 공식
* **Tx 출력 Raw:** $(\text{Torque Output}[0] \times 256) + \text{Torque Output}[1]$
* **Ty 출력 Raw:** $(\text{Torque Output}[2] \times 256) + \text{Torque Output}[3]$
* **Tz 출력 Raw:** $(\text{Torque Output}[4] \times 256) + \text{Torque Output}[5]$

#### 2) 물리량 변환 공식 (단위: Newton-meters [Nm])
$$\text{토크 [Nm]} = \frac{\text{Torque Output Raw}}{500} - 50$$

---
*문서 내용 중 `xx`로 표시된 데이터 필드는 Reserved(예약 영역) 또는 무관한 값이므로 처리 시 무시(Don't care)하시면 됩니다.*
