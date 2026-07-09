# UGV SDK 코드베이스 분석 리포트

> 작성일: 2026-06-25
> 대상: `zeroworks-robotics/ugv_sdk` (westonrobot/ugv_sdk 포크)
> 분석 범위: 전체 아키텍처 · CAN 프로토콜 구현 · fork 변경 이력 · 버그/리스크

---

## 1. 개요

AgileX / Weston Robot 이동 로봇 플랫폼용 **C++ SDK**. CAN(및 UART)을 통해 로봇에 주행 명령을 보내고
상태(state)를 수신하는 인터페이스를 제공한다.

| 항목 | 값 |
|---|---|
| 버전 (CMake) | `0.8.0` |
| 버전 (package.xml) | `0.1.6` |
| CHANGELOG 최신 | `0.1.5` (정지) |
| 환경 | x86_64 / arm64, Ubuntu 18.04~22.04, ROS Melodic/Noetic/Foxy/Humble |
| 의존성 | ASIO(비동기 I/O), GoogleTest(서브모듈) |
| 지원 로봇 | Scout(2.0/Mini/Omni), Hunter(1.0/2.0), Bunker, Tracer, Ranger(Mini 1.0/2.0/3.0/표준), Titan |

---

## 2. 아키텍처 (4계층)

```
┌─ Public API ─────────  include/ugv_sdk/mobile_robot/*.hpp   (ScoutRobot, RangerRobot …)
│   런타임 enum으로 프로토콜 선택 → 내부 템플릿 인스턴스에 위임 (어댑터)
├─ Robot Base ─────────  include/ugv_sdk/details/robot_base/*.hpp
│   AgilexBase<ParserType>  + 로봇별 Base(ScoutBase, RangerBase …)
│   상태 관리(뮤텍스 보호), 명령 송신, CAN RX 디스패치
├─ Protocol ───────────  include/ugv_sdk/details/protocol_v1|v2/ , src/protocol_v1|v2/
│   ParserBase<Version> ← ProtocolV1Parser<Limits> / ProtocolV2Parser
│   CAN frame ⇄ AgxMessage 인코딩/디코딩 (저수준은 C 파일)
└─ Transport ──────────  include/ugv_sdk/details/async_port/
    AsyncCAN(SocketCAN), AsyncSerial(UART) — ASIO 이벤트 루프 스레드
```

### 핵심 설계 패턴

- **컴파일타임 정책 주입**: 프로토콜·로봇 종류를 템플릿 `AgilexBase<ParserType>`로 정적 바인딩 → 제로 오버헤드.
  V1 파서는 `ProtocolV1Parser<RobotLimits>`로 로봇별 속도/각도 제한을 컴파일타임에 적용.
- **런타임 어댑터**: 공개 `ScoutRobot(ProtocolVersion, is_mini)` 생성자가 enum에 따라 `ScoutBaseV1`/`ScoutBaseV2`를
  생성하고 `RobotCommonInterface*`로 위임 → 사용자는 템플릿 복잡도를 알 필요 없음.
- **콜백 기반 RX**: `AsyncCAN`이 ASIO 이벤트 루프 스레드에서 프레임 수신 → `ParseCANFrame()` →
  파서 디코딩 → 상태 그룹별 뮤텍스 잠금 업데이트.
- **하드웨어 우회 서브클래스**: `RangerMiniV1Base`(펌웨어 모션모드 버그), `RangerMiniV2Base`(BMS 전압 스케일 0.01V)
  등 펌웨어 quirk를 코어 수정 없이 오버라이드로 패치.
- **상태 3분할**: core / actuator / common-sensor를 별도 뮤텍스로 분리해 락 경합 완화 (`agilex_base.hpp:198-208`).

### 데이터 흐름

- **명령**: `robot.SetMotionCommand(v, ω)` → `AgilexBase::SendMotionCommand()` → `AgxMessage` 구성 →
  파서 `EncodeMessage()` → `AsyncCAN::SendFrame()`
- **수신**: ASIO 스레드가 프레임 수신 → 파서 `DecodeMessage()` → `UpdateRobotCoreState()` 등 뮤텍스 보호 업데이트 →
  `robot.GetRobotState()`로 복사본 반환

---

## 3. CAN 프로토콜 구현 (검증됨)

### 주요 CAN ID (V2)

| 메시지 | V1 ID | V2 ID | 방향 | 주요 필드 |
|---|---|---|---|---|
| Motion Command | 0x130 | 0x111 | TX | linear/angular/lateral/steering |
| System State | 0x151 | 0x211 | RX | vehicle_state, control_mode, battery_voltage, error_code |
| Motion State | 0x131 | 0x221 | RX | linear/angular/lateral vel, steering angle |
| Light Command | 0x140 | 0x121 | TX | enable, front/rear mode + custom |
| Actuator HS (V2) | — | 0x251–0x258 | RX | rpm, current×0.1A, pulse_count(int32) |
| Actuator LS (V2) | — | 0x261–0x268 | RX | driver_voltage×0.1V, driver/motor temp |
| BMS Basic | — | 0x361 | RX | soc%, soh%, **voltage×0.01V**, current×0.1A, temp×0.1℃ |
| Version Req/Resp | — | **0x4a1 (양방향)** | TX/RX | 펌웨어 버전 88바이트 (11프레임). 헤더의 `0x411`은 실펌웨어 미사용 — §4-A 참조 |

### 스케일 / 엔디안

- **모션 명령(V2)**: 각 16비트 signed 필드를 `/1000` → mm/s, mrad/s, mrad.
  와이어는 **빅엔디안**(byte[0]=상위바이트). `struct16_t`는 `#ifdef`로 호스트 바이트오더만 전환하고,
  디코드는 `low | (high<<8)`로 명시 재조합 → 결과 정상. *(Ranger Mini 3.0 매뉴얼 p.18-19로 교차검증)*
- **체크섬(V1=V2 동일식)**: `(id&0xff) + (id>>8) + dlc + Σ data[0..dlc-2]`, 마지막 바이트에 저장.
- **모터 ID 유도**: `motor_id = can_id − CAN_MSG_ACTUATOR1_*_STATE_ID`.

### V1 vs V2 핵심 차이

| 항목 | V1 | V2 |
|---|---|---|
| 모션 명령 단위 | **퍼센트** int8 (−100~100) | **SI 단위** raw/1000 (mm/s, mrad/s, mrad) |
| 바이트 순서 | **빅엔디안** (data[2]=high) | **빅엔디안** (byte[0]=high) — V1과 동일 |
| 체크섬 | 디코드 시 **검증함** (`v1.c:20-24`) | 함수는 있으나 **디코드 검증 안 함** |
| 모터 피드백 | 4모터, 단일 프레임 | 8모터, HS+LS 분리 |

---

## 4. 발견된 버그 / 리스크 (코드 직접 검증)

### 🔴 0. Ranger Mini 3.0 배터리 전압 10배 과대 보고 — **실하드웨어로 확정**

**근거 1 (매뉴얼)**: Ranger Mini 3.0 User Manual p.34 — BMS Basic `0x361`의 battery voltage **단위 = 0.01 V**.

**근거 2 (실섀시 실측, 2026-07-09)**: `can0`에서 수동 관측
```
0x361 raw 4920  → ×0.01 = 49.2 V  ✅
                → ×0.1  = 492.0 V ❌  (수정 전 SDK가 보고했을 값)
0x211 raw 497   → ×0.1  = 49.7 V  ✅  (교차 확인, 0.1V 단위)
```

- 공통 파서 `src/protocol_v2/agilex_msg_parser_v2.c:262-265`: 전압을 `raw × 0.1f`로 디코드.
- `RangerMiniV2Base`(`ranger_base.hpp:104-127`): **추가 `× 0.1f`** → 최종 `× 0.01` = 매뉴얼과 일치 ✅
  (주석: "RM2 BMS voltage data follows unit: 0.01V").
- `RangerMiniV3Base`(`ranger_base.hpp:103`): `using RangerMiniV3Base = RangerBase;` — **RangerBase를 그대로 alias**.
  RangerBase의 `GetCommonSensorState`(`:81-100`)는 전압을 보정 없이 통과 → 최종 `× 0.1` 유지.

**결과**: Ranger Mini 3.0가 실제 48V를 **480V로 보고**(10배). V2에 있는 0.01V 보정이 V3에 누락.
**수정**: `RangerMiniV3Base`를 alias 대신 `RangerMiniV2Base`처럼 voltage `× 0.1f` 보정 클래스로 분리.
(current 0.1A·temperature 0.1°C는 V3도 통과 정상 — voltage 단위만 다름.)

### ✅ A. 버전 요청 CAN ID — **버그 아님 (실하드웨어로 반증)**

> **정정 (2026-07-09):** 아래는 최초 분석 시의 판단이며, **틀렸습니다.**
> 실섀시(Ranger Mini 3.0, HW `H-V1.3-1`, SW `S-V6.0-8250218`) 프로브 결과:
> ```
> 4A1#01  ->  0x4a1로 11프레임 × 8바이트 = 88바이트 (버전 blob)
> 411#01  ->  무응답
> ```
> **펌웨어는 `0x4a1` 하나로 요청·응답을 모두 처리합니다.** 따라서 `RequestVersion()`이
> `0x4a1`을 하드코딩한 것은 **정상**이고, `0x411`로 "고치면" 실기기에서 버전 조회가 깨집니다.
> 헤더의 `CAN_MSG_VERSION_REQUEST_ID = 0x411` 매크로 **이름이 오해를 부른 것**입니다.
>
> **교훈:** 헤더를 근거로 가설을 세우고, **그 헤더로 만든 시뮬레이터로 "검증"**했기에
> 순환 논증이었습니다. 하드웨어만이 가릴 수 있었습니다.
> 재현 도구: `ranger_can_sim/scripts/probe_version_id.sh <iface>` (읽기 전용).
>
> **실제 결함은 따로 있음:** 파서의 `AgxMsgVersionRequest` 인코더가 `VersionRequestFrame`을
> 초기화 없이 `memcpy` (미정의 동작). → ugv_sdk PR #4에서 수정.

<details><summary>최초 분석 (틀린 판단, 기록용)</summary>

#### A. 버전 요청을 응답 CAN ID로 전송 — 기능 미동작 의심

`include/ugv_sdk/details/robot_base/agilex_base.hpp:247`
```cpp
frame.can_id = ((uint32_t)0x4a1);   // 0x4a1 = CAN_MSG_VERSION_RESPONSE_ID
```
헤더 정의(`src/protocol_v2/agilex_protocol_v2.h:102-103`):
```c
#define CAN_MSG_VERSION_REQUEST_ID  ((uint32_t)0x411)   // 요청은 이쪽이어야 함
#define CAN_MSG_VERSION_RESPONSE_ID ((uint32_t)0x4a1)   // 응답
```
**요청을 응답 ID(0x4a1)로 전송 중.** 펌웨어가 0x4a1을 요청으로도 수용하지 않는 한 `RequestVersion()`은
항상 타임아웃 → 빈 문자열 반환 가능성이 높다. 매직넘버 하드코딩 + 파서 `EncodeMessage` 우회 문제도 동반.
(upstream 원본 코드의 문제로 보임.) → **실하드웨어 검증 후 `0x411`로 수정 권장.**
※ Ranger Mini 3.0 매뉴얼 CAN 섹션(p.15-34)에는 버전 요청/응답 프레임이 문서화되어 있지 않아 매뉴얼만으로는 검증 불가.


</details>

### 🟡 B. LS 디코드의 union 멤버 오기 — 현재는 우연히 동작

`src/protocol_v2/agilex_msg_parser_v2.c:180` (LS 분기인데 HS 멤버에 기록)
```c
msg->body.actuator_hs_state_msg.motor_id =      // actuator_ls_state_msg 가 맞음
    rx_frame->can_id - CAN_MSG_ACTUATOR1_LS_STATE_ID;
```
`body`는 union이고 두 구조체 모두 `motor_id`가 **offset 0의 첫 멤버**(`agilex_message.h:104,120`)라 메모리가 겹쳐
결과적으로 정상 동작한다. 그러나 레이아웃 변경 시 깨지는 잠재 복붙 버그. → `actuator_ls_state_msg.motor_id`로 수정 권장.

### 🔴 C-2. Bunker V1 actuator_state 배열 오버플로우 — 빌드 경고로 발견

`bunker_base.hpp:59`의 복사 루프는 `i < 3`(`:56`)으로 도는데, V1용 대상 배열
`BunkerActuatorState::actuator_state`는 `[2]`로 선언됨(`bunker_interface.hpp:34`).
→ `i=2`에서 **범위 밖 쓰기**. GCC 경고: *"writing 60 bytes into a region of size 40 overflows"*.
Bunker 3모터 수정 시 `actuator_hs_state`/`actuator_ls_state`만 `[3]`으로 늘리고 V1 배열을 누락한 것.
**수정**: `bunker_interface.hpp:34` `actuator_state[2]` → `[3]` (HS/LS와 동일하게). (현재 ranger 브랜치 범위 밖 — 별도 처리 권장.)

### ✅ C. Bunker 모터 수정 — 정상 (충돌 없음)

커밋 시간순: `c2f2aa0`(2025-09-02, base를 ACTUATOR2로) → `3f35979`(2025-12-23, ACTUATOR1로 되돌림, **현재 상태**).
현재 `:156,:181`은 `ACTUATOR1` 기준이라 0x251→0, 0x252→1, 0x253→2 이고 배열 `[3]`·루프 `i<3`과 정합. **올바름.**
Bunker는 모터 3개 구성으로 확정.

### 🟡 D. 모션 명령 angular/steering 동시 입력 처리

`agilex_base.hpp:103-105` (V1 경로): `angular_vel`와 `steering_angle` 중 **절댓값이 큰 쪽만** 채택.
두 값이 동시에 의미를 가지면 작은 쪽이 조용히 무시됨. (정상적으로는 둘 중 하나만 non-zero라는 전제.)

---

## 5. 빌드 / 유지보수 리스크

- **버전 메타데이터 3중 불일치**: CMake `0.8.0` / package.xml `0.1.6` / CHANGELOG `0.1.5`(정지). 릴리스 추적 불가.
- **테스트 사각지대**: 단위 테스트는 V1 Scout/Hunter 명령 2개뿐(`test/unit_tests/`).
  V2 전체·Ranger·Titan·Bunker·Tracer·BMS·체크섬 검증이 미커버 → 위 A/B 같은 버그가 안 잡힌 원인.
- **BMS 인코더 미구현**: `agilex_msg_parser_v2.c`의 `AgxMsgBmsBasic` 인코드가 `// TODO`로 빈 frame 전송.
- **복붙 기반 신규 로봇 추가의 위험**: `e87b71c`(tracer)가 Scout 복붙으로 인한 템플릿 하드코딩
  (`AgilexBase<ProtocolV2Parser>`→`<ParserType>`)·변수명 오기(`scout`→`Tracer`)·누락 메서드를 한 번에 수정.

---

## 6. Fork 변경 이력 (zeroworks)

| 커밋 | 날짜 | 내용 |
|---|---|---|
| `c3dfaf4` | 2026-04-07 | Ranger Mini 3.0 제품 목록 추가 (문서) |
| `3f35979` | 2025-12-23 | Bunker 모터 읽기 수정 (motor-id base → ACTUATOR1) |
| `c2f2aa0` | 2025-09-02 | Bunker CAN 데이터 업데이트 (이후 3f35979가 정정) |
| `e87b71c` | 2025-08-25 | Tracer 버그 수정 (템플릿/변수명/누락 메서드/CMake 타깃) |
| `b0b7499` | 2025-08-06 | ROS2 버그 수정 (Ranger Variant enum 도입, 레거시 생성자 제거) |
| `fb81632` / `62f4539` | 2025-07 | BMS 메시지(basic/extended) 추가 |
| `fd728d4` | 2025-04-02 | Ranger Mini V3 지원 (ranger_base 대규모 리팩터) |

---

## 7. 권장 후속 작업 (우선순위)

> **진행 상태 (2026-07-09):** 1·3·4 완료 후 `main`에 병합(PR #1, #2). 2번은 반증되어 PR #3 클로즈,
> 대신 근거 주석 + 인코더 미초기화 수정으로 PR #4 병합. **실하드웨어(Ranger Mini 3.0, HW `H-V1.3-1`,
> SW `S-V6.0-8250218`)로 최종 검증 완료.**

1. ✅ **[0] Ranger Mini 3.0 전압 10배 버그** — `RangerMiniV3Base`를 `RangerMiniV2Base` 상속으로 변경해 0.01V 보정 재사용 (`ranger_base.hpp`). **완료.**
2. ❌ **[A] 버전요청 ID** — **버그 아님.** 실하드웨어가 `0x4a1`(요청·응답 공용)임을 확인. 수정 시도(PR #3)는 클로즈. 대신 근거 주석 + 인코더 미초기화 수정(PR #4).
3. ✅ **[B] union 멤버 오기** — `agilex_msg_parser_v2.c:180` `actuator_hs_state_msg`→`actuator_ls_state_msg`. **완료.**
4. ✅ **회귀 테스트** — `test/unit_tests/utest_ranger_bms_voltage.cpp` 추가(V3 전압 0.01V 보정 검증). **완료.**
5. ⏳ **버전 메타데이터 동기화** — CMake/package.xml/CHANGELOG 정렬.
6. ⏳ **BMS 인코더** 구현(시뮬레이터/테스트 목적이라면).

> ✅ 빌드/테스트 검증 완료 — `cona_icn_ros2:0.0.1` 컨테이너(ROS2 Humble)에서
> `cmake -DBUILD_TESTING=ON` 빌드 성공, `utests` 실행 시 신규 테스트 2개 PASS.
> (기존 `HunterV1CommandTest.MotionAngularCommandTest` 1개 실패는 clean 트리에서도 동일한 **사전 존재 실패**로 본 변경과 무관.)
> 단, 실하드웨어 검증은 여전히 권장(특히 안전 관련 BMS 전압).
> 빌드 중 `ranger_base.hpp`의 `M_PI` 미선언 노출 → `<cmath>` include 추가로 해결.

---

## 8. 실섀시 버스 전수 감사 (2026-07-09)

Ranger Mini 3.0 (HW `H-V1.3-1`, SW `S-V6.0-8250218`)에 `can0`(500k)로 연결해
**10초간 송신 없이** 수동 캡처한 결과(8,943 프레임).

### 관측된 피드백 ID와 실측 주기

| ID | 실측 주기 | 매뉴얼 | SDK 디코드 | 시뮬레이터 |
|---|---|---|---|---|
| `0x211` 시스템 | ~21 ms | 20 ms | ✅ | ✅ |
| **`0x212`** | **~2000 ms** | **미문서화** | **❌ 정의 없음** | ❌ |
| `0x221` 모션 | ~21 ms | 20 ms | ✅ | ✅ |
| `0x231` 조명 상태 | ~21 ms | 20 ms | ✅ | **❌ 미송신** |
| `0x241` RC | ~21 ms | 20 ms | ✅ | ✅ |
| `0x251`–`0x258` HS | ~21 ms | 20 ms | ✅ | ✅ |
| `0x261`–`0x268` LS | ~105 ms | 100 ms | ✅ | ✅ |
| `0x271` 조향각 | ~21 ms | 20 ms | ✅ | ✅ |
| `0x281` 휠속도 | ~21 ms | 20 ms | ✅ | ✅ |
| `0x291` 모션모드 | ~21 ms | 20 ms | ✅ | ✅ |
| `0x311` 전륜 odom | ~21 ms | 20 ms | ✅ | ✅ |
| **`0x312` 후륜 odom** | ~21 ms | 20 ms | **❌ 정의 없음** | ✅ |
| `0x361` BMS | **~244 ms** | 500 ms | ✅ | ✅ |
| `0x362` BMS 확장 | **~238 ms** | 500 ms | ✅ | ✅ |

### 발견 1 — SDK가 후륜 오도메트리(`0x312`)를 버린다 🔴

헤더에 `CAN_MSG_ODOMETRY_ID = 0x311`만 있고 `0x312` 정의가 없습니다. 실섀시는 전/후륜
오도메트리를 각각 보내지만 SDK는 전륜만 디코드하고 후륜은 무시합니다.

```
0x311 전륜: left 22059 mm   right 25650 mm
0x312 후륜: left 22048 mm   right 25737 mm
```

### 발견 2 — 미문서화 프레임 `0x212` 🟡

```
0x212  [4]  00 07 01 02        (~2초 주기, 값 고정)
```

SDK 헤더에도 AgileX 매뉴얼에도 없습니다. 용도 불명이며 SDK는 무시합니다.
후속 관측에서도 4바이트 전부 `00 07 01 02`로 **고정**이었습니다(정지 상태). 값 패턴이
버전/설정 식별자처럼 보이나 변동 관측이 없어 단정 불가. **SDK가 읽지 않아 기능 영향 없음.**

> **주의:** 최초 프로브 때 `411#01000...` 직후 `0x212`가 한 번 잡혀 "응답"으로 오인했으나,
> 위 수동 캡처로 **원래 주기적으로 나오는 프레임**임이 확인되었습니다. 우연의 일치였습니다.

### 발견 2b — `0x362` 뒤 4바이트 리버스 엔지니어링 🟡

매뉴얼은 `0x362`를 **4바이트**(Alarm 1/2, Warning 1/2)로 정의하지만 실섀시는 **8바이트**를 보냅니다.

| 바이트 | 관측 | 의미 |
|---|---|---|
| [0] Alarm Status 1 | `0x00` | 알람 없음 (과전압/저전압/고온/저온/방전과전류 비트) |
| [1] Alarm Status 2 | `0x00` | 알람 없음 (충전과전류) |
| [2] Warning Status 1 | `0x00` | 경고 없음 |
| [3] Warning Status 2 | `0x00` | 경고 없음 |
| [4~7] | `00 00 08 06` | **매뉴얼 미정의** |

- 앞 4바이트 전부 `0` = 배터리 정상(SOC 63%, 49.1V, 25℃, 무결함)과 정합 → **매뉴얼의 4바이트
  알람/경고 정의는 정확**함을 확인.
- 뒤 4바이트는 매뉴얼에 없으나 **SDK가 `BmsExtendedFrame`에서 `reserved0~3`으로 선언해 무시**함
  (dlc=8 처리). → 데이터 손실·버그 아님.
- ⚠️ `byte[6]`이 세션 간 `0x02`→`0x08`로 **변동** 관측(`byte[7]`=`0x06` 고정). 뒤 바이트는
  상수가 아니라 내부 카운터/상태로 추정되나, 표본 2개뿐이라 정체 미상.

### 발견 3 — 매뉴얼과 실측이 다른 항목 🟡

| 항목 | 매뉴얼 | 실측 |
|---|---|---|
| BMS `0x361`/`0x362` 주기 | 500 ms | **~240 ms** |
| `0x362` 길이 | 4 바이트 | **8 바이트** (앞 4=알람/경고, 뒤 4=미문서·SDK reserved, §발견 2b) |
| `0x291` 길이 | 2 바이트 | **3 바이트** |

### 스케일 검증 — SDK 디코드 전부 실측과 일치 ✅

```
0x261 LS  : driver_volt 49.0 V (x0.1)  driver_temp 36 C  motor_temp 30 C  state 0x40(ENABLED)
0x361 BMS : soc 66 %  soh 100 %  volt 49.10 V (x0.01)  cur -0.6 A  temp 24.5 C
0x211     : battery 49.7 V (x0.1)   <- BMS와 교차 확인
```

### 명령(송신) 경로 검증 — 바퀴 공중 상태 ✅

실섀시로 SDK 명령 API를 확인(저속·단시간, 바퀴 공중):

- `EnableCommandedMode()`: `control_mode` standby(0) → CAN(1). **단, standby 상태에서만 성공** —
  SWB 중간(RC)이거나 리모컨 신호 손실이면 RC가 최우선이라 CAN 명령이 차단됨.
- `SendMotionCommand()`: 전진/후진/조향/게걸음(parallel)/스핀 모두 바퀴 반응 + 정지 정상.
- `SetMotionMode()`: 4개 모드 전환 모두 동작. 조향각이 실측 기하와 일치
  (SPIN/PARK `±935`, dual-ackerman 앞뒤 반대).
- `SetLightCommand()` / `DisableLightControl()`: CAN으로 점등/소등 확인 (`0x231` front_mode 반영).

### 발견 4 — 모드 전환은 원자적이지 않다 (motion 끊김) 🔴

ACKERMAN→SPIN 전환 중 스핀 속도 명령을 **끊김없이 계속 보내면서** `0x291`/`0x271`/`0x281`을
고속 관측:

```
t=2.87  motion_mode ACKER->SPIN (즉시)   조향=0      휠속도=0
t=3.18                                   조향=606    휠속도=0   (조향 모터가 X자로 이동 중)
t=3.48                                   조향=935    휠속도=78  (재배치 완료 후에야 회전 시작)
```

- **전환 명령 → 실제 구동까지 ~0.6초 공백.** 그동안 속도 명령을 계속 보내도 **무시**됨.
  전환은 "① 조향 모터를 새 형상으로 물리 재배치(~0.6s) → ② 그 후 구동" 순서로 진행.
  매뉴얼(`0x291`: "전환 중엔 속도 제어 명령에 응답하지 않음")과 정합.
- ⚠️ **`0x291` byte[1] `mode_changing` 플래그는 내내 `0`이었음** (이 펌웨어는 세팅 안 하거나
  20ms 주기 사이로 빠짐). 즉 `motion_mode` 번호는 즉시 바뀌지만 `mode_changing`만으로는
  "조향 재배치 완료" 여부를 알 수 없다. **`mode_changing`을 믿고 전환 완료를 판단하면 안 됨.**

**실사용 권장:** 모드 전환 후 주행 명령 전에 (a) ~0.5–1s 대기하거나 (b) `0x271` 조향각이
목표에 도달했는지 확인. 그렇지 않으면 전환 직후 명령이 조용히 무시됨.

---

## 부록: 공식 매뉴얼 대조 (Ranger Mini 3.0 User Manual V1.0.0, 2024.06)

`~/Documents/Manual/ranger-mini-mini-3.0-user-manual.pdf` §3.2 CAN Communication Protocol 기준.

| 프레임 | ID | 주기 | SDK 일치 여부 |
|---|---|---|---|
| Motion Command | 0x111 | 20ms (timeout 500ms) | ✅ (/1000, 빅엔디안) |
| Motion Feedback | 0x221 | 20ms | ✅ |
| Motor HS feedback | 0x251–258 | 20ms | ✅ (current 0.1A) |
| Motor LS feedback | 0x261–268 | 100ms | ✅ (volt 0.1V, drv temp 1℃) |
| Steering angle (5–8) | 0x271 | 20ms | ✅ (0.001 rad) |
| Wheel speed (1–4) | 0x281 | 20ms | ✅ (mm/s) |
| Motion mode state | 0x291 | 20ms | ✅ |
| Odometry F/R | 0x311 / 0x312 | 20ms | mm, int32 |
| RC feedback | 0x241 | 20ms | ✅ |
| BMS Basic | 0x361 | 500ms | ⚠️ **voltage 0.01V** — V3 미보정 (위 §4-0) |
| BMS Extended | 0x362 | 500ms | alarm/warning 비트 |
| Set motion mode | 0x141 | — | 0=Ackerman,1=oblique,2=spin,3=park |
| Light command | 0x121 | 20ms | Ranger은 0=off/1=on만 유효 |
| Control mode | 0x421 | — | 0=standby,1=CAN |
| 전류/전압 모드 전환 | 0x423 | — | X-parking에서만 가능 |
| Error clear | 0x441 | — | byte0=클리어 대상 코드 |

- 모션명령 유효범위(매뉴얼): linear ±2000mm/s(조향>20°시 ±1000), angular ±3259(0.001rad/s),
  조향 inner angle Ackerman ±698 / oblique ±1571.
- 모터 번호: 1=우전/2=우후/3=좌후/4=좌전(구동), 5~8=조향. SDK `motor_id = id−0x251`과 정합.
- 버전 요청/응답 프레임은 매뉴얼에 **미문서화** → 버그 A는 별도 검증 필요.
