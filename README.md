# Hey Doopal — 시각장애인 맞춤형 음성 제어 로봇 보조 시스템

> 음성 명령만으로 협동로봇(Doosan M0609)이 주변 사물을 탐색·파지해 시각장애인 사용자 손에 전달하는 Voice-to-Robot 보조 시스템 (4인 팀 프로젝트, 2026.07.16~2026.07.29)

이 저장소는 팀 프로젝트 "Hey Doopal" 중 **로봇 제어 파트**(본인 직접 구현)를 담고 있다. 원본 팀 저장소: [sonnanlo2125-a11y/simbongsa](https://github.com/sonnanlo2125-a11y/simbongsa)

## Key Features

- **능동형 재탐색 (Cone Scan)**: TCP를 고정한 채 Rx/Ry 8자세로 회전하며 물체를 재탐색하는 스캔 로직 — 1회 스캔에 물체가 탐지되지 않아도 포기하지 않고 다각도로 재시도
- **ROS2 Action 기반 통신**: Goal-Feedback-Result 구조의 `FindOrder`/`FindTargetOrder` 커스텀 액션으로 로봇 제어 노드와 비전/DB 파트 간 비동기 통신을 처리
- **Compliance/Force 기반 안전 파지**: 접촉력(|Fz|) 기준으로 파지 판정을 수행해 다양한 크기의 물체를 안전하게 그립
- **음성 명령 처리(직접 구현, 별도 관리)**: WAKEUP/BUSY/CONFIRM/SLEEP 상태머신과 LangChain 기반 자연어 파라미터 추출, openWakeWord 커스텀 시동어 모델 — 해당 모듈 코드는 이 저장소에는 포함되어 있지 않음

## Tech Stack

| 구분 | 내용 |
|---|---|
| 언어 | Python |
| 로봇 미들웨어 | ROS2 Humble (rclpy, Action/Service) |
| 로봇 | Doosan Robotics M0609 (DR_init / DSR 제어 API) |
| 통신 | 커스텀 ROS2 Action/Service (`hey_doopal_msg`) |

## Repository 구조

| 경로 | 설명 |
|---|---|
| `robot_control/robot_control/cone_scan.py` | Cone Scan 능동형 재탐색 로직 |
| `robot_control/robot_control/test_robot_control3.py` | 로봇 제어 통합 노드 (최신 버전) |
| `hey_doopal_msg/action/` | `FindOrder`, `FindTargetOrder` — 비전 탐색 결과를 주고받는 커스텀 Action |
| `hey_doopal_msg/srv/` | `ScanRequest`, `GripBoundingBox`, `VoiceKeyword`, `GetDbData` — 파트 간 인터페이스 |

## 담당 파트

본 프로젝트는 4인 팀 프로젝트다. 음성 인식(자연어 명령 처리)과 로봇 제어(본 저장소)는 본인이 직접 구현했고, Object Detection(YOLO 기반 비전 인식)과 DB·UI(손 추적·웹 인터페이스)는 팀원이 담당했다.
