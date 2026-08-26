# robot_control (로봇 제어)

시각장애인 보조 로봇 **두팔이** 프로젝트 중 로봇팔(Doosan M0609, `DSR_ROBOT2`) 제어측 구현. `voice_control`(본인 구현)에서 파싱된 명령을 서비스로 받아, `detection`(협업 파트)의 YOLO/MediaPipe 인식 결과를 이용해 물체를 찾고, 집고, 사용자 손이나 지정 위치까지 옮기는 Pick-and-Place 전체를 담당한다.

핵심은 두 가지다. 하나는 물체가 바로 안 보일 때 TCP를 원뿔 모양으로 기울여가며 다시 찾는 **Cone Scan**(`cone_scan.py`), 다른 하나는 그리퍼 힘 피드백과 로봇 Compliance 제어를 결합한 **안전 파지/배치**다.

## 구성 파일

- **`robot_control/cone_scan.py`**: `ConeScanner` 클래스. 중심 자세에서 시작해 `Rx/Ry`를 원형으로 기울인 8개 자세를 순차 이동하며 YOLO Action(`FindOrder`)에 물체가 잡힐 때까지 스캔한다. `robot_control`/`detection` 어느 쪽 Action Client(`find_order`, `find_target_order`, `find_hand_order`)에도 재사용 가능하도록 액션 클라이언트를 주입받는 구조다.
- **`robot_control/test_robot_control.py`**: 가장 이른 스냅샷. 스캔만 있고 그리퍼/파지 로직이 없다.
- **`robot_control/test_robot_control1.py`**: `AdaptiveGripper`(Modbus 기반) 그리퍼 클래스와 `run_adaptive_grip`(bbox 면적 기반 힘 계산)이 처음 추가된 버전. 단일 `find_order` 액션과 `/get_keyword` 서비스(`VoiceKeyword`)로 명령을 받는다.
- **`robot_control/test_robot_control2.py`**: 물체용/손용 Action을 `/find_target_order`, `/find_hand_order`로 분리하고, 내려놓기에 Force Compliance(`place_with_compliance`)를 추가, UI/DB 쪽에 상태를 알리는 `Bool` 토픽들을 추가한 버전.
- **`robot_control/test_robot_control3.py`**: 이 스냅샷에서 가장 최신/완성형. `test_robot_control2.py`와 거의 동일한 구조에 `GetDbData` 서비스 클라이언트(Redis DB 조회, `ui_db` 팀원 파트와 연동 예정)가 추가돼 있다.
- **`robot_control/__init__.py`**: 빈 파일.
- **`resource/T_gripper2camera.npy`**: 4×4 gripper→camera 핸드아이 캘리브레이션 행렬(순수 NumPy 배열, 회전 오차가 작고 z 오프셋 약 -228.8mm인 것으로 보아 손목 근접 카메라 캘리브레이션 결과로 추정). 실제로 이 파일을 로드하는 코드는 `robot_control` 안에는 없다(아래 "알려진 이슈" 참고).
- **`test/test_copyright.py`, `test/test_flake8.py`, `test/test_pep257.py`**: `ament` 표준 린트 테스트. 프로젝트 고유 로직은 없다.
- **`package.xml`, `setup.py`, `setup.cfg`**: `ament_python` 빌드 메타데이터. `setup.py`의 `entry_points`는 아래 "버전별 진화" 절 참고.

> 파일명이 전부 `test_`로 시작하지만, 이들은 pytest 단위 테스트가 아니라 **실제 ROS2 노드 진입점**이다(`setup.py`의 `entry_points`에 `console_scripts`로 등록되어 `ros2 run robot_control test1_robot_control` 형태로 실행된다). 진짜 ament 단위 테스트는 `test/` 디렉터리 쪽이다. 이 네이밍은 오해 소지가 있고, 패키지 루트에서 pytest를 그대로 돌리면 이 파일들도 테스트 모듈로 수집을 시도해 import 시점에 `rclpy.init()`과 실기 연결을 시도할 위험이 있다.

## 핵심 설계 1: Cone Scan (능동형 재탐색)

`ConeScanner.scan(center_pose, target_name)`은 다음 순서로 동작한다.

1. `movel(center_pos, ...)`로 중심 자세로 먼저 이동한다.
2. YOLO Action 서버(`action_client`, 예: `/find_target_order`)가 5초 안에 응답 가능한지 확인하고, `FindOrder.Goal(target_name=...)`을 비동기로 보낸다.
3. Goal이 승인되면(`goal_callback`) 결과 대기용 `threading.Event`(`result_event`)를 건다.
4. `scan_point_count`(=8)개의 자세를 계산해 순서대로 `amovel`(비동기 이동)로 옮겨가며, 각 자세로 이동하는 동안 `result_event`가 set되는지, `check_motion() == 0`(이동 완료)이 되는지를 30ms 간격으로 폴링한다. 결과가 오면 즉시 반환하고, 8개 자세를 다 돌아도 결과가 없으면 진행 중인 Goal을 취소(`cancel_goal_async`)하고 중심 자세로 복귀한 뒤 `None`을 반환한다.

스캔 자세는 중심 자세의 `rx, ry`만 원형으로 흔드는 방식이다.

```python
theta = 2.0 * math.pi * index / scan_point_count      # index = 0..7
tilt_rx = scan_tilt_angle * math.cos(theta)             # scan_tilt_angle = 30.0(deg)
tilt_ry = scan_tilt_angle * math.sin(theta)
pose = [x, y, z, rx + tilt_rx, ry + tilt_ry, rz]        # x,y,z,rz는 중심 자세 그대로
```

즉 `x, y, z`(TCP 위치)는 고정한 채 `rx, ry`(자세)만 반지름 30˚짜리 원을 그리며 흔드는 것으로, 카메라가 원뿔면을 훑듯 물체를 재탐색하게 만든다. 실제 사용되는 상수(`test_robot_control3.py` 기준)는 다음과 같다.

```python
SCAN_TILT_ANGLE = 30.0
SCAN_POINT_COUNT = 8
SCAN_VELOCITY = [20, 10]      # movel/amovel의 [속도, 회전속도]
SCAN_ACCELERATION = [40, 20]
```

## 핵심 설계 2: 안전 그리퍼 파지 + Compliance 배치

파지는 2단계로 나뉜다.

**1) 접근 후 bbox 기반 적응형 그립 (`run_adaptive_grip`)**

`GripBoundingBox` 서비스로 받은 픽셀 bbox 크기(`bbox_w`, `bbox_h`)와 카메라 깊이(`dist`)로부터 대략적인 실제 면적을 추정하고, 그 값에 비례해 그리퍼 힘을 정한다.

```python
pixel_area = bbox_w * bbox_h
real_area_estimate = pixel_area * (dist ** 2)
K = 0.00001
force = min(int(150 + (real_area_estimate * K)), 400)   # 하한 150, 상한 400
```

이후 `AdaptiveGripper.move_gripper(width_val=0, force_val=force)`로 Modbus 레지스터에 `[force, width, 16]`을 쓰고(주소 0, unit 65), 2초 대기 후 상태 레지스터(주소 268)의 특정 비트를 읽어 파지 성공 여부를 로그로만 남긴다(`get_status()[0] == 1`이면 성공).

**2) 내려놓기 Compliance 제어 (`place_with_compliance`, `test_robot_control2/3.py`)**

```python
def place_with_compliance(self, press_force=10.0, force_threshold=8.0, timeout=5.0):
    set_ref_coord(DR_BASE)
    task_compliance_ctrl(stx=[3000, 3000, 500, 200, 200, 200])
    set_desired_force(fd=[0, 0, -press_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0.5, mod=DR_FC_MOD_REL)
    while rclpy.ok():
        if check_force_condition(DR_AXIS_Z, min=force_threshold, ref=DR_BASE):
            contacted = True; break
        if time.time() - start_time > timeout:
            break
        time.sleep(0.02)
    release_force(time=0.2)
    release_compliance_ctrl()
```

목표 위치 위에서 `-Z` 방향으로 10N을 가하며 하강 대신 힘 순응 제어로 내려가다가, Z축 외력이 **8N**을 넘으면(`force_threshold=8.0`) 바닥/테이블에 닿은 것으로 보고 멈춘다. 역량기술서의 "Compliance 8N"은 이 `force_threshold=8.0` 값과 정확히 일치한다(코드에서 직접 확인). `timeout=5.0`초 안에 접촉이 감지되지 않으면 `contacted=False`로 빠져나가고, 이 경우 물체를 쥔 채로 `place_with_compliance`가 `False`를 반환한다(아래 "알려진 이슈" 참고).

## 파이프라인

```
[voice_control] GetKeyword → (VoiceKeyword 서비스, 필드 불일치 있음: voice_control/README.md 참고)
     │
     ▼
[robot_control] command_callback(request.target, request.goal)
     │ threading.Thread로 execute_robot_task 실행
     ▼
 목표가 "hand"(또는 goal 미지정)면:
   hand_scanner(ConeScanner, action=/find_hand_order) → detection/hand_detection_node.py
                                                          (MediaPipe 손 위치, 협업 파트)
 목표가 위치 이름이면:
   CONFIG[goal_name] 좌표를 그대로 사용
     │
     ▼
 target_names 각각에 대해:
   target 위치 위로 이동(movel, +100mm) → 그리퍼 오픈(width=1000,force=200)
     │
     ▼
   /grip_bounding_box 서비스 호출 → detection/object_detection_node.py (YOLO 근접 bbox+depth)
     │ is_find == False면
     ▼
   target_scanner(ConeScanner, action=/find_target_order) 재탐색 → detection/object_detection_node.py
     │
     ▼
   movel(grip_coordinate) → run_adaptive_grip() → AdaptiveGripper(Modbus, 실제 그리퍼)
     │
     ▼
 goal이 hand면: movel(goal) → /arrived_goal(Trigger) 호출 → detection/hand_tracking_node.py
 goal이 위치면: movel(goal 위) → place_with_compliance() (Force/Compliance, DSR_ROBOT2)
```

## 버전별 진화와 entry_points

`setup.py`의 `console_scripts`는 다음과 같다.

```python
entry_points={
    'console_scripts': [
        'robot_control = robot_control.robot_control:main',
        'test_robot_control = robot_control.test_robot_control:main',
        'test1_robot_control = robot_control.test_robot_control1:main',
        'test2_robot_control = robot_control.test_robot_control2:main',
        'mock_yolo = robot_control.mock_yolo_server:main',
        'mock_voice = robot_control.mock_voice:main',
    ],
},
```

이 중 실제로 파일이 존재하는 항목은 `test_robot_control`, `test1_robot_control`, `test2_robot_control` 셋뿐이다. `robot_control.robot_control`, `robot_control.mock_yolo_server`, `robot_control.mock_voice`는 대응 파일이 없어(`__init__.py`는 빈 파일) 이 스냅샷 그대로 빌드하면 콘솔 스크립트 등록 단계에서 깨진다. 그리고 정작 이 스냅샷에서 가장 완성도 높은 `test_robot_control3.py`는 `entry_points`에 전혀 등록되어 있지 않아, `colcon build` 후 `ros2 run robot_control ...`로 바로 실행할 진입점이 없다.

## 설정 스키마

별도 JSON 설정 파일 없이, 각 노드 파일 상단에 로봇 좌표를 파이썬 딕셔너리로 하드코딩한다(`test_robot_control3.py` 기준).

```python
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY = 60
ACCELERATION = 60

CONFIG = {
    "SCAN_WAYPOINT1": [434.7, 21.07, 552.72, 63.44, -179.21, 62.54],
    "SCAN_WAYPOINT2": [434.7, -187.14, 552.72, 63.44, -179.21, 62.54],
    "SCAN_WAYPOINT3": [431.95, -392.07, 419.67, 147.77, 180, -33.61],
    "pos1": [744.33, -127.61, 228.65, 169.90, -142.46, 97.08],
    "pos2": [342.82, 171.74, 395.16, 26.39, -176.26, -1.66],
    "HAND_SCAN": [445.29, -23.52, 533.56, 90, -90, -90],
    "drink": [420.59, -73.95, 256.60, 60.13, -179.84, -111.53],
    "airpods": [420.59, -73.95, 256.60, 60.13, -179.84, -111.53],
    "mouse": [420.59, -73.95, 256.60, 60.13, -179.84, -111.53],
}
```

좌표는 `posx` 형식(`x, y, z`[mm], `rx, ry, rz`[deg])이며 티칭으로 얻은 값을 그대로 박아 넣은 것으로 보인다. `drink`/`airpods`/`mouse`가 전부 동일한 좌표를 쓰는 것도 실물 배치를 정확히 반영했다기보다 데모용 임시값에 가깝다. 코드 상단 주석에 "나중에 데이터베이스 또는 통신으로 바뀔 부분 ctrl+f 'CONFIG'"라고 명시돼 있어, `GetDbData` 서비스(아래 참고)로 대체할 계획이었음을 알 수 있다.

## 토픽 / 서비스 / 액션 스키마

**액션 `FindOrder`** (`hey_doopal_msg/action/FindOrder`, `/find_order`·`/find_target_order`·`/find_hand_order`에서 동일 타입으로 재사용)

```
# Goal
string target_name
---
# Result
bool found
float64[3] coordinate  # base_link 기준 [x, y, z], mm
string message
---
# Feedback
string state
```

`ConeScanner`가 `feedback_callback`에서 `feedback.state`를 로그로만 사용하고 별도 로직 분기는 없다. 참고로 `hey_doopal_msg`에는 `FindTargetOrder.action`(거의 동일한 구조)도 정의·빌드되어 있지만, `robot_control`/`detection` 어디에서도 import되지 않는 미사용 인터페이스다.

**서비스**

- `/get_keyword` (`VoiceKeyword`: req `target, goal` / res `accepted`) — 음성측이 호출하는 명령 입구. `voice_control` 쪽 구현과의 필드 불일치 문제는 `voice_control/README.md`에 정리했다(로봇측 코드 자체는 `target`/`goal`만 쓰므로 인터페이스 정의와 일치한다).
- `/grip_bounding_box` (`GripBoundingBox`: req `target` / res `coordinate[3], bbox_width, bbox_height, camera_depth_z`) — `detection` 제공.
- `/yolo_scan_request` (`ScanRequest`: req `waypoint_id` / res `success, message, detected_count`) — 테이블 전체 스캔용, `detection` 제공.
- `/assistive/get_db_data` (`GetDbData`: req `data_type, name` / res `success, json_data, message`) — `test_robot_control3.py`의 `get_objet_from_db()`(오타: object가 아니라 objet)에서 요청 객체만 만들 뿐, 실제로 호출하는 곳이 없다. `ui_db` 연동이 아직 배선되지 않은 상태다.
- `/ungrip`, `/arrived_goal` (`std_srvs/Trigger`) — 각각 그리퍼 강제 해제, 손 접근 완료 신호.

**토픽 (모두 `test_robot_control2.py`부터 등장, QoS: RELIABLE/VOLATILE, depth 10)**

| 토픽 | 실제 속성명 | 콜백에서 참조하는 이름 | 상태 |
|---|---|---|---|
| `/table_scan_finished` | `pub_table_scan` | `pub_table_scan` | 정상 |
| `/hand_scan_start` | `pub_hand_start` | `pub_hand_start` | 정상 |
| `/hand_scan_finished` | `pub_hand_finish` | `pub_hand_finish` | 정상 |
| `/task_completed` | `pub_task_completed` | `pub_task_done` | **불일치** |
| `/table_rescan_started` | `pub_re_scan_start` | `pub_table_rescan_start` | **불일치** |
| `/table_rescan_finished` | `pub_re_scan_finish` | `pub_table_rescan_finish` | **불일치** |
| `/say` | `pub_say` | (참조 없음) | 미사용 |

불일치 3건은 아래 "알려진 이슈"에서 자세히 다룬다.

## 알려진 이슈 (코드에서 실제로 확인됨)

- **Publisher 속성명 불일치로 인한 크래시**: `__init__`에서 만든 실제 속성은 `self.pub_task_completed`, `self.pub_re_scan_start`, `self.pub_re_scan_finish`인데, `ungrip_callback`은 `self.pub_task_done.publish(...)`를, `execute_robot_task`의 재탐색 분기는 `self.pub_table_rescan_start`/`self.pub_table_rescan_finish`를 참조한다. 존재하지 않는 속성이라 `/ungrip` 서비스가 호출되거나 bbox 재탐색이 발생하면 그 콜백/스레드가 `AttributeError`로 죽는다. `test_robot_control2.py`에는 이전 이름으로 주석 처리된 줄(`# self.pub_task_done = ...`, `# 수정` 표시)이 남아 있어, 속성명을 리네임하면서 호출부를 놓친 흔적으로 보인다.
- **다중 타깃 파지 회귀**: `test_robot_control1.py`까지는 `for target_name in target_names:` 루프 안에서 타깃마다 이동→파지→내려놓기를 전부 반복했다. `test_robot_control2/3.py`에서 Compliance 배치 로직을 추가하면서 `movel(grip_coordinate, ...)` 이후의 파지/배치 블록이 `for`문과 같은 들여쓰기로 빠져나와(루프 밖) 있다. 그 결과 YOLO bbox 조회는 여전히 타깃마다 반복되지만, 실제로 옮겨서 잡고 놓는 것은 루프의 **마지막 타깃 하나뿐**이다. 예외 없이 조용히 발생하는 논리 회귀다.
- **빈 타깃 리스트일 때 크래시**: 같은 이유로 `target_names`가 빈 리스트면(`request.target`이 빈 문자열/공백일 때) `for` 루프가 한 번도 안 돌아 `grip_coordinate`가 정의되지 않은 채 루프 밖 `movel(grip_coordinate, ...)`이 실행되어 `UnboundLocalError`가 난다.
- **미사용 DB 연동 코드**: `get_objet_from_db()`와 `self.db_client`(`/assistive/get_db_data`)는 `test_robot_control3.py`에 정의만 되어 있고 어디서도 호출되지 않는다. `CONFIG` 하드코딩을 Redis DB(`ui_db` 팀원 파트) 조회로 대체하려던 미완성 작업으로 보인다.
- **죽은 리소스 위치**: `resource/T_gripper2camera.npy`(4×4 handeye 캘리브레이션 행렬)는 `robot_control` 패키지 리소스로 배포되지만, 이를 로드하는 코드는 `detection` 패키지(`hand_detection_node.py` 등, 협업 파트)에만 있다. 패키징 위치와 실제 소비 주체가 다르다.
- **`AdaptiveGripper` 클래스 3중 복붙**: `test_robot_control1/2/3.py` 세 파일에 동일한 `AdaptiveGripper` 클래스(생성자, `move_gripper`, `get_status`, `close_connection`)가 토씨 하나 다르지 않게 반복되어 있다. 공용 모듈로 분리되지 않았다.
- **깨진/미등록 entry_points**: 위 "버전별 진화" 절 참고 — `robot_control`, `mock_yolo_server`, `mock_voice`는 대응 파일이 없고, 가장 최신 버전인 `test_robot_control3.py`는 등록조차 안 돼 있다.
- **`package.xml`에 실사용 의존성 선언 없음**: `rclpy`, `std_msgs`, `std_srvs`, `hey_doopal_msg` 등을 실제로 import하지만 `<depend>`는 하나도 없고 `<test_depend>`(ament 린트용)만 있다. rosdep 관점에서 의존성 메타데이터가 비어 있는 상태다.

## 알려진 제약 / TODO

- 로봇 좌표가 전부 파이썬 코드에 하드코딩돼 있다(`CONFIG` 딕셔너리). 미완성 상태인 `GetDbData` 연동이 완료되면 이 부분이 Redis 조회로 대체될 것으로 보인다.
- `run_adaptive_grip()`의 힘 계산 상수(`K=0.00001`, 하한 150, 상한 400)와 Modbus `force_val` 레지스터 값의 실제 단위(그램힘인지, 그리퍼 자체 0~1000 스케일인지)가 코드/주석 어디에도 설명돼 있지 않다.
- `AdaptiveGripper`의 Modbus `unit=65`, 상태 레지스터 주소 `268`, 명령 레지스터 주소 `0` 등은 그리퍼 제조사 레지스터 맵을 그대로 하드코딩한 값으로 보이며, 이름 있는 상수로 분리되어 있지 않다.
- `place_with_compliance()`가 5초 타임아웃 안에 접촉을 감지하지 못하면 `contacted=False`로 빠져나가 "내려놓기 실패"만 로깅하고 종료한다 — 이때 그리퍼는 물체를 쥔 채로 남고, 재시도나 안전 위치 복귀 같은 복구 로직은 없다.
- `place_with_compliance()`의 접촉 판정 루프는 `time.sleep(0.02)`(20ms) 폴링으로 구현돼 있어 실시간성이 완벽히 보장되지는 않는다(순수 폴링 방식의 한계).
