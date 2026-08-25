# Hey Doopal — 시각장애인 맞춤형 음성 제어 로봇 보조 시스템

> 음성 명령만으로 협동로봇(Doosan M0609)이 주변 사물을 탐색·파지해 시각장애인 사용자 손에 전달하는 Voice-to-Robot 보조 시스템 (4인 팀 프로젝트, 2026.07.16~2026.07.29)

이 저장소는 팀 프로젝트 "Hey Doopal"의 통합 코드베이스다. **로봇 제어(`src/robot_control`)와 음성 명령 처리(`src/voice_control`)는 본인이 직접 구현**했고, **Object/Hand Detection(`src/detection`)과 DB·UI(`src/ui_db`)는 팀원이 구현**한 코드를 프로젝트 전체 구조 이해를 돕기 위해 함께 포함했다. 원본 팀 저장소: [sonnanlo2125-a11y/simbongsa](https://github.com/sonnanlo2125-a11y/simbongsa) (브랜치별 팀원 작업 분리)

## Key Features

- **능동형 재탐색 (Cone Scan)**: TCP를 고정한 채 Rx/Ry 8자세로 회전하며 물체를 재탐색하는 스캔 로직 — 1회 스캔에 물체가 탐지되지 않아도 포기하지 않고 다각도로 재시도
- **ROS2 Action 기반 통신**: Goal-Feedback-Result 구조의 `FindOrder`/`FindTargetOrder` 커스텀 액션으로 로봇 제어 노드와 비전/DB 파트 간 비동기 통신을 처리
- **Compliance/Force 기반 안전 파지**: 접촉력(|Fz|) 기준으로 파지 판정을 수행해 다양한 크기의 물체를 안전하게 그립
- **음성 명령 처리 (직접 구현)**: WAKEUP → BUSY → CONFIRM → SLEEP 상태머신, 커스텀 TFLite 웨이크워드 모델(`MicController`), STT + 자연어 파라미터 추출(`get_keyword`)로 로봇 제어 서버에 목표물/목적지를 전달

## System Architecture

빨간색 박스가 본인이 직접 구현한 `robot_control`·`voice_control` 파트다. 파란색 박스는 팀원이 구현한 `detection`·`ui_db` 파트로, 이번 정리에서 참고용으로 저장소에 함께 포함했다 (`src/` 이하 구조는 [Repository 구조](#repository-구조) 참고).

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'background': '#ffffff', 'fontFamily': 'Apple SD Gothic Neo, Malgun Gothic, sans-serif', 'fontSize': '14px', 'primaryTextColor': '#1a1a1a', 'lineColor': '#555555'}, 'flowchart': {'curve': 'basis', 'htmlLabels': true}}}%%
flowchart LR
    classDef own fill:#FDECEC,stroke:#C24343,stroke-width:1.6px,color:#611E1E,rx:8,ry:8;
    classDef team fill:#EAF2FB,stroke:#3B6EA5,stroke-width:1.6px,color:#1B3A5C,rx:8,ry:8;
    classDef infra fill:#F1F1F1,stroke:#5A5A5A,stroke-width:1.2px,color:#2B2B2B,rx:8,ry:8;

    subgraph VOICE[" 🎙️  voice_control · 본인 직접 구현 "]
        direction TB
        MIC["MicController<br/>커스텀 TFLite 웨이크워드"]:::own
        STT["stt.py / get_keyword.py<br/>STT + 파라미터 추출"]:::own
        VSRV["robot_control_server<br/>WAKEUP→BUSY→CONFIRM→SLEEP"]:::own
    end

    subgraph DET[" 👁️  detection · 팀원 구현 "]
        direction TB
        OD["object_detection_node<br/>YOLO Bounding Box"]:::team
        HD["hand_detection_node<br/>hand_tracking_node<br/>MediaPipe 3D 손 좌표"]:::team
    end

    subgraph DBUI[" 🗄️  ui_db · 팀원 구현 "]
        direction TB
        FLASK["app.py (Flask)<br/>redis_store"]:::team
        BRIDGE["ros_object_bridge"]:::team
    end

    subgraph CORE[" 🤖  robot_control · 본인 직접 구현 "]
        direction TB
        TSN["TargetScanNode<br/>(robot_control_node)<br/>명령 오케스트레이션"]:::own
        CONE["ConeScanner<br/>능동형 재탐색<br/>(8방향 콘 스캔)"]:::own
        GRIP["AdaptiveGripper<br/>Modbus TCP 그리퍼 제어"]:::own
    end

    ARM["🦾 Doosan M0609<br/>DSR_ROBOT2 API"]:::infra

    VSRV -->|"/get_keyword (VoiceKeyword.srv)"| TSN
    TSN -->|"/find_target_order, /find_hand_order<br/>(FindOrder action)"| OD
    TSN -->|"/yolo_scan_request (ScanRequest)"| OD
    TSN -->|"/grip_bounding_box (GripBoundingBox)"| OD
    TSN -->|"/find_hand_order (FindOrder action)"| HD
    TSN -->|"/assistive/get_db_data (GetDbData)"| FLASK
    BRIDGE -.->|"인식 결과 동기화"| FLASK
    TSN --> CONE
    CONE -->|"FindOrder action (동일 서버)"| OD
    TSN --> GRIP
    TSN -->|"movel / task_compliance_ctrl"| ARM
```

## 동작 플로우

### A. 음성 명령 → 물체 탐색·파지 → 전달

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'background': '#ffffff', 'primaryColor': '#ffffff', 'primaryBorderColor': '#000000', 'primaryTextColor': '#000000', 'lineColor': '#000000', 'edgeLabelBackground':'#ffffff'}, 'flowchart': {'curve': 'linear'}}}%%
flowchart TD
    classDef proc fill:#ffffff,stroke:#000000,stroke-width:1.4px,color:#000000;
    classDef dec fill:#ffffff,stroke:#000000,stroke-width:1.4px,color:#000000;
    classDef term fill:#000000,stroke:#000000,stroke-width:1.4px,color:#ffffff;
    classDef own fill:#f2f2f2,stroke:#000000,stroke-width:1.4px,color:#000000;

    START(["음성 명령 인식<br/>(src/voice_control)"]):::term
    CALL["① /get_keyword 서비스 호출<br/>target, goal 전달"]:::proc
    ACCEPT["② command_callback<br/>즉시 accepted=True 반환<br/>실제 작업은 백그라운드 스레드로 위임"]:::own
    ISHAND{"③ goal이 비어있거나<br/>'hand'인가"}:::dec
    HANDSCAN["④ ConeScanner(hand_scanner).scan()<br/>/find_hand_order 액션으로 손 위치 탐색<br/>(상세 · 플로우 B)"]:::own
    FIXEDGOAL["④' CONFIG[goal_name]에서<br/>고정 목적지 좌표 조회"]:::own
    APPROACH["⑤ 목표물 100mm 위로 이동 → 그리퍼 오픈"]:::own
    BBOX["⑥ /grip_bounding_box 서비스 호출<br/>정밀 좌표 + bbox 크기 획득"]:::proc
    FOUND{"⑦ 물체를 찾았는가"}:::dec
    RESCAN["⑧ ConeScanner(target_scanner).scan()<br/>/find_target_order 액션으로 재탐색<br/>(상세 · 플로우 B)"]:::own
    MOVEGRIP["⑨ 정밀 좌표로 이동"]:::own
    ADAPTGRIP["⑩ run_adaptive_grip()<br/>bbox 면적·깊이로 그립력 계산 → Modbus로 파지<br/>그립 감지 상태 비트 확인"]:::own
    LIFT["⑪ 100mm 상승"]:::own
    DELIVER{"⑫ 목적지가 손인가<br/>고정 위치인가"}:::dec
    HAND_DELIV["손으로 이동 → /arrived_goal Trigger 호출"]:::own
    PLACE["place_with_compliance()<br/>힘 제어로 접촉 감지될 때까지 하강 → 그리퍼 오픈"]:::own
    END(["작업 완료"]):::term

    START --> CALL --> ACCEPT --> ISHAND
    ISHAND -- 예 --> HANDSCAN --> APPROACH
    ISHAND -- 아니오 --> FIXEDGOAL --> APPROACH
    APPROACH --> BBOX --> FOUND
    FOUND -- 아니오 --> RESCAN --> MOVEGRIP
    FOUND -- 예 --> MOVEGRIP
    MOVEGRIP --> ADAPTGRIP --> LIFT --> DELIVER
    DELIVER -- 손 --> HAND_DELIV --> END
    DELIVER -- 고정 위치 --> PLACE --> END
```

### B. Cone Scan 능동형 재탐색 상세

```mermaid
%%{init: {'theme': 'base', 'themeVariables': {'background': '#ffffff', 'primaryColor': '#ffffff', 'primaryBorderColor': '#000000', 'primaryTextColor': '#000000', 'lineColor': '#000000', 'edgeLabelBackground':'#ffffff'}, 'flowchart': {'curve': 'linear'}}}%%
flowchart TD
    classDef proc fill:#ffffff,stroke:#000000,stroke-width:1.4px,color:#000000;
    classDef dec fill:#ffffff,stroke:#000000,stroke-width:1.4px,color:#000000;
    classDef term fill:#000000,stroke:#000000,stroke-width:1.4px,color:#ffffff;
    classDef own fill:#f2f2f2,stroke:#000000,stroke-width:1.4px,color:#000000;

    IN(["ConeScanner.scan(center_pose, target_name) 진입"]):::term
    MOVE["① center_pose로 이동"]:::own
    GOAL["② FindOrder 액션 목표 전송<br/>(target_name)"]:::own
    LOOP["③ i = 1..8 포즈 순회 준비<br/>θ = 2π·i/8"]:::own
    POSE["④ 틸트 포즈 계산 후 비동기 이동<br/>tilt_rx = 30·cos(θ), tilt_ry = 30·sin(θ)"]:::own
    WAIT["⑤ 30ms 간격으로<br/>모션 완료 + 액션 결과 폴링"]:::own
    DETECTED{"⑥ 이번 포즈에서<br/>탐지됐는가"}:::dec
    RETURN_FOUND["⑦ 탐지 좌표 즉시 반환<br/>(남은 포즈 스킵)"]:::own
    MORE{"⑧ 8개 포즈를<br/>모두 순회했는가"}:::dec
    CANCEL["⑨ 액션 goal 취소<br/>center_pos로 복귀 → None 반환"]:::own
    OUT(["결과 반환<br/>(좌표 또는 None)"]):::term

    IN --> MOVE --> GOAL --> LOOP --> POSE --> WAIT --> DETECTED
    DETECTED -- 예 --> RETURN_FOUND --> OUT
    DETECTED -- 아니오 --> MORE
    MORE -- 아니오 --> POSE
    MORE -- 예 --> CANCEL --> OUT
```

## Tech Stack

| 구분 | 내용 |
|---|---|
| 언어 | Python |
| 로봇 미들웨어 | ROS2 Humble (rclpy, Action/Service) |
| 로봇 | Doosan Robotics M0609 (DR_init / DSR 제어 API) |
| 음성 | 커스텀 TFLite 웨이크워드, STT, 자연어 파라미터 추출 |
| 비전 (팀원) | YOLOv8, MediaPipe |
| DB·UI (팀원) | Flask, Redis |
| 통신 | 커스텀 ROS2 Action/Service (`hey_doopal_msg`, `od_msg`) |

## Repository 구조

| 경로 | 설명 |
|---|---|
| `src/robot_control/` | 로봇 제어 ROS2 Action Server — Cone Scan, 안전 파지, DSR 제어 (**본인 직접 구현**) |
| `src/voice_control/` | 음성 명령 처리 — 웨이크워드, STT, 자연어 파라미터 추출, 상태머신 (**본인 직접 구현**) |
| `src/hey_doopal_msg/` | 팀 공용 커스텀 ROS2 Action/Service 인터페이스 (`FindOrder`, `ScanRequest`, `GripBoundingBox`, `VoiceKeyword`, `GetDbData` 등) |
| `src/detection/` | YOLO 물체 인식 · MediaPipe 손 추적 노드 (팀원 구현, 구조 이해를 위해 포함) |
| `src/ui_db/` | Flask 기반 재고/이벤트 관리 웹 UI · Redis 연동 (팀원 구현, 구조 이해를 위해 포함) |

> `src/detection/`, `src/ui_db/` 는 팀원이 구현했으며, 학습된 YOLO 가중치(`.pt`) 등 대용량 바이너리는 포함하지 않았다. 전체 팀원별 원본 브랜치는 [원본 팀 저장소](https://github.com/sonnanlo2125-a11y/simbongsa/branches/all)에서 확인할 수 있다.

## 담당 파트

본 프로젝트는 4인 팀 프로젝트다. **음성 인식(자연어 명령 처리)과 로봇 제어**는 본인이 직접 구현했고 (`src/robot_control`, `src/voice_control`), **Object Detection(YOLO 기반 비전 인식)과 DB·UI(손 추적·웹 인터페이스)**는 팀원이 담당했다 (`src/detection`, `src/ui_db`).
