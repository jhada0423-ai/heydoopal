# voice_control (음성 인식 및 상태 제어)

시각장애인 보조 로봇 **두팔이** 프로젝트 중 음성 입력측 구현. 커스텀 시동어("헤이 두팔") 감지 → Whisper STT → LangChain 기반 슬롯 추출을 거쳐, 상태 머신(WAKEUP/CONFIRM/SLEEP)으로 로봇 제어측(`robot_control`, 본인 구현)에 명령을 전달한다.

이 폴더에는 실제로 쓰이는 `voice_processing` 패키지와, 어디서도 참조되지 않는 `od_msg` 패키지가 함께 들어 있다.

## 구성 파일

- **`voice_processing/voice_processing/wakeup_word.py`**: `WakeupWord` 클래스. openWakeWord + 커스텀 tflite 모델로 시동어를 감지한다.
- **`voice_processing/voice_processing/MicController.py`**: `MicConfig`(dataclass)와 `MicController`. PyAudio 스트림 열기/녹음/저장을 담당한다.
- **`voice_processing/voice_processing/stt.py`**: `STT` 클래스. OpenAI Whisper(`whisper-1`)로 음성을 텍스트로 변환한다.
- **`voice_processing/voice_processing/get_keyword.py`**: `GetKeyword` 노드. 시동어 감지 → STT → LangChain 슬롯 추출 → 상태 머신 → `robot_control` 호출까지의 메인 루프.
- **`voice_processing/voice_processing/robot_control_server.py`**: `RobotControlServer` 노드. `voice_keyword_service`를 열어 음성 명령을 받는 쪽인데, 실제 로봇을 움직이지 않는 **시뮬레이션 목업**이다(아래 참고).
- **`voice_processing/voice_processing/test_mic.py`**: PyAudio 입력 장치 목록을 출력하는 7줄짜리 유틸리티 스크립트(노드 아님, `entry_points`에도 없음).
- **`voice_processing/resource/hey_doopal_final.tflite`**: 실제 사용되는 커스텀 시동어 모델.
- **`voice_processing/resource/hello_rokey_8332_32.tflite`**: 코드 어디에서도 참조되지 않는 리소스(부트캠프 예제용 "헬로 로키" 시동어 모델로 추정).
- **`od_msg/`**: `SrvDepthPosition.srv`(`string target` → `float64[] depth_position`) 하나만 정의된 별도 ROS2 패키지. `voice_processing` 코드 어디에서도 import되지 않는다. `README_branch_source.md`에 "simbongsa / 두산로보틱스 로키 협동2 D-2조"라고 적혀 있어 다른 브랜치나 다른 조 실습에서 넘어온 흔적으로 보인다.

## 핵심 설계 1: 시동어 감지 (openWakeWord)

`WakeupWord.is_wakeup()`은 마이크 버퍼를 읽어 리샘플링한 뒤 모델에 넣는다.

```python
audio_chunk = np.frombuffer(self.stream.read(self.buffer_size, ...), dtype=np.int16)
audio_chunk = resample_poly(audio_chunk, up=1, down=3).astype(np.int16)  # 48kHz → 16kHz
outputs = self.model.predict(audio_chunk, threshold=0.1)
confidence = outputs[self.model_name]
if confidence > 0.5:
    return True
```

마이크는 48kHz로 녹음하지만 `openWakeWord`는 16kHz 입력을 기대하므로 `resample_poly(up=1, down=3)`으로 3분의 1로 다운샘플링한다(48000/3=16000, 계산이 맞다). 다만 `model.predict(...)`에 넘기는 `threshold=0.1`과, 그 결과를 실제로 게이팅하는 `confidence > 0.5`는 서로 다른 값이며, 이 둘의 관계는 코드/주석만으로는 명확하지 않다(둘 다 매직 넘버로 하드코딩).

`set_stream()`에서는 모델이 요구하는 최소 프레임 수만큼 0으로 채운 프레임을 미리 넣어(`preprocessor(np.zeros(1280, ...))`) 버퍼를 예열시키는데, 이 부분도 하드코딩된 `1280`을 쓴다.

## 핵심 설계 2: 상태 머신 (WAKEUP / CONFIRM / SLEEP)

`GetKeyword.main_loop()`(0.1초 주기 타이머)는 `self.current_mode` 문자열에 따라 분기한다. 실제로 코드에 등장하는 모드 문자열은 3가지뿐이다.

```python
if self.current_mode == "WAKEUP":
    if not self.wakeup_word.is_wakeup(): return
    if time.time() - self.last_command_time < self.cooldown_duration: return   # cooldown_duration = 5
    self.current_mode = "SLEEP"                       # 로컬에서 먼저 SLEEP으로 전환(중복 감지 방지)
    output_message = self.stt.speech2text(self.mic_controller.stream)
    objects, targets = self.extract_keyword(output_message)
    if objects and targets:
        self.send_to_robot(objects[0], targets[0])     # 첫 번째 항목만 사용
        self.last_command_time = time.time()
    else:
        self.current_mode = "WAKEUP"                   # 파싱 실패 시 원복

elif self.current_mode == "CONFIRM":
    output_message = self.stt.speech2text(self.mic_controller.stream)
    if "받았어" in output_message or "받" in output_message:
        self.send_to_robot("ACK_RECEIVED", "hand_done")
        self.current_mode = "SLEEP"

elif self.current_mode == "SLEEP":
    pass
```

모드를 밖에서 바꾸는 쪽은 `robot_control_server.py`다. `handle_voice_command()`가 명령을 받자마자 `change_voice_mode("SLEEP")`으로 음성 인식을 멈추고, `goal == "hand"`면 완료 후 `"CONFIRM"`으로, 아니면 `"WAKEUP"`으로 되돌린다.

> 역량기술서에는 상태가 `WAKEUP/BUSY/CONFIRM/SLEEP` 4단계로 기술되어 있지만, 저장소 전체를 검색해도 `"BUSY"` 문자열은 어디에도 없다. 코드상 실제로 쓰이는 모드 문자열은 `WAKEUP`, `CONFIRM`, `SLEEP` 3가지뿐이며, 로봇이 움직이는 동안은 `robot_control_server.py`가 모드를 `SLEEP`으로 유지시키는 구간이 사실상 "BUSY"에 해당하는 것으로 보인다(별도 문자열로 구분되지는 않음). 코드를 기준으로 이 차이를 명시해 둔다.

## 핵심 설계 3: LangChain 기반 슬롯 추출

```python
self.llm = ChatOpenAI(model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key)
self.prompt_template = PromptTemplate(input_variables=["user_input"], template=prompt_content)
self.lang_chain = self.prompt_template | self.llm
```

프롬프트에 도구 리스트를 하드코딩(`airpods, cable, drink, mouse, pos1, pos2, pos3`)하고, `"[도구1 도구2 ... / pos1 pos2 ...]"` 형식으로만 답하도록 지시한다. 함수 호출(function calling)이나 구조화 출력(structured output) 없이 순수 텍스트 프롬프트로 슬롯을 뽑아내는 방식이라, 파싱은 전적으로 LLM이 형식을 지켜준다는 가정에 의존한다.

```python
def extract_keyword(self, output_message):
    response = self.lang_chain.invoke({"user_input": output_message})
    result = response.content
    if "/" not in result:
        return [], []
    obj_str, target_str = result.strip().split("/")   # "/"가 정확히 1개라고 가정
    ...
```

`"/"`가 있는지만 확인할 뿐, 2개 이상 등장하는 경우는 걸러내지 않는다. LLM이 형식을 어기고 슬래시를 여러 번 출력하면 `result.strip().split("/")`이 3개 이상의 값을 반환해 `obj_str, target_str = ...` 언패킹에서 `ValueError`로 죽는다.

**교차 모듈 불일치 (도구 목록)**: 프롬프트가 허용하는 도구/목적지 이름은 `airpods, cable, drink, mouse, pos1, pos2, pos3`이지만, `robot_control`의 `CONFIG` 딕셔너리(`test_robot_control3.py`)에 실제로 등록된 키는 `SCAN_WAYPOINT1~3, pos1, pos2, HAND_SCAN, drink, airpods, mouse`뿐이다. `cable`과 `pos3`는 로봇측에 아예 없다. 즉 사용자가 "케이블을 pos3에 놔줘"라고 말하면 LangChain은 정상적으로 `cable / pos3`를 뽑아내지만, `robot_control.execute_robot_task`는 `goal_name not in CONFIG`로 즉시 에러 로그를 내고 아무 동작 없이 종료하거나(목적지가 `pos3`인 경우), `target_coordinate`가 정의되지 않은 채 뒤 코드가 실행돼 `UnboundLocalError`가 날 수 있다(타깃이 `cable`인 경우). 두 모듈이 서로 다른 "정답 목록"을 갖고 있는 셈이다.

**교차 모듈 불일치 (다중 물체)**: 프롬프트 예시는 `"airpods cable / pos1"`처럼 여러 도구를 한 번에 지원하도록 설계돼 있고, `robot_control.command_callback`도 `request.target.strip().split()`으로 공백 구분된 여러 타깃 이름을 받게 짜여 있다. 하지만 `main_loop`는 `objects[0]`, `targets[0]`만 취해 **항상 하나만** 전송한다. 게다가 `robot_control` 쪽에도 마지막 타깃만 실제로 파지되는 독립적인 버그가 있다(`robot_control/README.md`의 "다중 타깃 파지 회귀" 참고). 즉 다중 물체 명령은 음성측과 로봇측 양쪽에서 각각 다른 이유로 끊겨 있어, 어느 한쪽만 고쳐서는 동작하지 않는다.

## 핵심 이슈: `VoiceKeyword.srv` 필드 불일치

실제 빌드되는 인터페이스 정의(`hey_doopal_msg/srv/VoiceKeyword.srv`)는 다음이 전부다.

```
# Request
string target
string goal
---
# Response
bool accepted
```

그런데 `get_keyword.py`와 `robot_control_server.py`는 이 인터페이스에 없는 필드를 읽고 쓴다.

```python
# get_keyword.py: send_to_robot()
request = VoiceKeyword.Request()
request.mode = self.current_mode      # ← Request에 mode 필드 없음
request.target = target_val
request.goal = goal_val

# get_keyword.py: handle_mode_change()
response.success = True               # ← Response에 success 필드 없음
response.message = f"Mode changed to {self.current_mode}"   # ← message 필드도 없음

# robot_control_server.py: change_voice_mode()
req = VoiceKeyword.Request()
req.mode = mode                        # ← 동일

# robot_control_server.py: handle_voice_command()
response.success = True
response.message = "Command processed successfully"
```

`rosidl`이 생성하는 Python 메시지/서비스 클래스는 `__slots__` 기반이라 정의에 없는 속성을 대입하면 자동으로 새 속성이 생기는 게 아니라 **즉시 `AttributeError`**가 난다. 즉 이 스냅샷 코드를 실제로 빌드된 `hey_doopal_msg`와 함께 실행하면, `send_to_robot()`이 호출되는 순간(첫 명령 인식 시점)과 `handle_mode_change()`/`handle_voice_command()`가 호출되는 순간 바로 크래시한다. `git log`상 `VoiceKeyword.srv`는 이 스냅샷에 처음이자 유일하게 커밋된 버전이라, 이전에 `mode`/`success`/`message` 필드가 있었다가 빠진 것도 아니다 — 인터페이스와 사용 코드가 애초부터 어긋나 있다.

## 핵심 이슈: `robot_control_server.py`는 로봇과 연결되지 않은 목업

```python
def handle_voice_command(self, request, response):
    ...
    self.change_voice_mode("SLEEP")
    if request.goal == "hand":
        # 예: 5cm 거리 감지 루프 가정...
        time.sleep(2)
        self.change_voice_mode("CONFIRM")
    else:
        time.sleep(3)
        self.change_voice_mode("WAKEUP")
```

주석에 그대로 "[여기에 로봇 제어 로직 시뮬레이션]"이라고 적혀 있듯, 이 노드는 실제 `DSR_ROBOT2`나 `robot_control` 패키지를 전혀 호출하지 않고 `time.sleep()`으로 로봇 동작 시간을 흉내만 낸다. 게다가 이 노드가 여는 서비스 이름(`voice_keyword_service`, `set_voice_mode_service`)은 `robot_control`이 실제로 여는 서비스 이름(`/get_keyword`)과도 다르다. 즉 필드 불일치 문제를 걷어내더라도, 이 스냅샷의 `voice_control`은 실제 로봇 노드에 배선되어 있지 않고 독립된 로컬 시뮬레이터 상태로 남아 있다.

## 설정 파일 스키마

별도 설정 파일 없이 `MicConfig`(dataclass, `MicController.py`)와 각 노드 안의 오버라이드 값으로 마이크를 구성한다.

```python
@dataclass
class MicConfig:
    chunk: int = 12000
    rate: int = 48000
    channels: int = 1
    record_seconds: int = 5
    fmt: int = pyaudio.paInt16
    device_index: int = 4          # 하드코딩된 장치 인덱스
    buffer_size: int = 24000
```

`get_keyword.py`는 이 기본값을 그대로 쓰지 않고 자체 값으로 덮어써서 `MicController`를 만든다.

```python
mic_config = MicConfig(chunk=3840, rate=48000, channels=1, record_seconds=5,
                        fmt=pyaudio.paInt16, device_index=4, buffer_size=3840)
```

`device_index=4`는 특정 PC에서 특정 시점에 확인한 마이크 장치 번호를 그대로 박아 넣은 값으로, 다른 컴퓨터나 USB 포트 구성이 바뀌면 깨진다(`test_mic.py`로 인덱스를 다시 확인해야 한다). API 키는 `.env` 파일에서 읽는다.

```python
ENV_PATH = os.path.join(PACKAGE_PATH, "resource", ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")
```

`resource/.env`는 이 저장소 스냅샷에는 포함돼 있지 않다(비밀키라 제외된 것으로 보인다). 그런데 `setup.py`의 `data_files`는 이 파일을 명시적으로 패키징 대상에 넣는다.

```python
(os.path.join('share', package_name, 'resource'), glob('resource/*') + ['resource/.env']),
```

`resource/.env`가 로컬에 없는 상태로 `colcon build`를 돌리면 `setuptools`가 존재하지 않는 파일을 데이터로 지정한 것이 되어 빌드가 깨질 수 있다(로컬에 `.env`를 먼저 만들어야 빌드가 통과한다는 뜻이며, 이 전제가 문서화돼 있지는 않다).

## 파이프라인

```
[wakeup_word.py] 48kHz 마이크 → 16kHz 리샘플 → openWakeWord(hey_doopal_final.tflite)
     │ confidence > 0.5
     ▼
[get_keyword.py] mode="WAKEUP" → STT.speech2text() (5초 녹음 → Whisper) → 텍스트
     │
     ▼
[get_keyword.py] extract_keyword() → LangChain(gpt-4o) → "도구 / 목적지" 파싱 → objects[0], targets[0]
     │ send_to_robot(target, goal)  (VoiceKeyword 서비스, 필드 불일치로 실제로는 크래시)
     ▼
[robot_control_server.py] handle_voice_command()
     │ change_voice_mode("SLEEP")
     │ goal == "hand"?
     ├─ 예: time.sleep(2) 후 change_voice_mode("CONFIRM")  → mode="CONFIRM"
     │        [get_keyword.py] STT로 "받았어" 대기 → 감지 시 send_to_robot("ACK_RECEIVED","hand_done")
     └─ 아니오: time.sleep(3) 후 change_voice_mode("WAKEUP")
     (실제 DSR_ROBOT2/robot_control 노드 호출 없음 — 시뮬레이션)
```

## 토픽 / 서비스 스키마

**`VoiceKeyword.srv`** (`hey_doopal_msg/srv/VoiceKeyword`)

```
string target
string goal
---
bool accepted
```

실제 사용 코드가 요구하는(그러나 정의에는 없는) 필드까지 포함하면 사실상 필요한 스키마는 다음에 가깝다: `target, goal, mode`(요청), `success, message`(응답, `accepted` 대신/추가로). 인터페이스를 코드에 맞게 정정하거나, 코드를 인터페이스에 맞게 정정하는 작업이 필요하다.

**서비스 이름**

| 이름 | 서버 | 클라이언트 | 비고 |
|---|---|---|---|
| `set_voice_mode_service` | `get_keyword.py`(`handle_mode_change`) | `robot_control_server.py`(`change_voice_mode`) | 같은 `voice_control` 안에서만 쓰임 |
| `voice_keyword_service` | `robot_control_server.py`(`handle_voice_command`) | `get_keyword.py`(`send_to_robot`) | `robot_control`의 `/get_keyword`와는 별개 |

**Whisper STT 호출**

ROS 메시지가 아니라 OpenAI API 직접 호출이다.

```python
transcript = self.client.audio.transcriptions.create(model="whisper-1", file=f)
```

`STT.speech2text()`는 `stream.read(3840, ...)`을 63회 반복해 오디오를 모은다. 48000Hz 기준 `63 * 3840 / 48000 ≈ 5.04초`인데, 코드 내 주석은 서로 다른 시간을 말한다: 함수 시작부의 `print`는 "5초 동안"이라고 하고, 반복문 위 주석은 "3초 동안 ... (48000Hz * 3초 / 3840 = 37.5회 반복)"이라고 설명하면서 정작 반복 횟수는 그 계산값(37.5)이 아니라 하드코딩된 `63`을 쓴다. 주석 두 개, 실제 값 하나가 서로 다른 상태다.

## 알려진 이슈 (코드에서 실제로 확인됨)

- **`VoiceKeyword.srv` 필드 불일치로 인한 크래시**: `request.mode`, `response.success`, `response.message`가 실제 `.srv` 정의에 없어 `AttributeError`가 발생한다(위 "핵심 이슈" 절 참고). 이 스냅샷 그대로는 첫 명령 인식 시점에 죽는다.
- **`robot_control_server.py`는 실제 로봇과 분리된 목업**: `time.sleep()` 기반 시뮬레이션이며, 서비스 이름도 `robot_control` 패키지의 실제 서비스(`/get_keyword`)와 다르다.
- **역량기술서의 `BUSY` 상태가 코드에 없음**: 실제 모드 문자열은 `WAKEUP`/`CONFIRM`/`SLEEP` 3가지뿐이다.
- **다중 물체 명령 미지원**: `extract_keyword()`가 여러 도구를 뽑아내도 `main_loop`는 첫 번째 항목만 전송한다(`robot_control` 쪽의 독립적인 버그와 합쳐져 이중으로 막혀 있다).
- **도구 목록 불일치**: LangChain 프롬프트의 허용 목록(`cable`, `pos3` 포함)과 `robot_control.CONFIG`의 실제 키가 다르다.
- **LLM 출력 파싱이 형식 가정에 의존**: `"/"` 존재 여부만 확인하고, 슬래시가 2개 이상이면 언패킹에서 `ValueError`로 죽는다. function calling/structured output 없이 순수 프롬프트 지시만으로 파싱한다.
- **`resource/.env` 부재 시 빌드 위험**: `setup.py`의 `data_files`가 저장소에 없는 `resource/.env`를 명시적으로 참조한다.
- **죽은 리소스**: `resource/hello_rokey_8332_32.tflite`는 어디서도 로드되지 않는다.
- **미사용 패키지**: `od_msg`(`SrvDepthPosition.srv`) 전체가 `voice_processing` 코드에서 참조되지 않는다.
- **주석-코드 불일치**: `stt.py`의 녹음 시간 관련 주석(3초/37.5회)과 실제 동작(하드코딩 63회 ≈ 5.04초)이 서로 다르다.
- **하드코딩된 마이크 장치 인덱스**: `MicConfig.device_index=4`가 고정돼 있어 다른 환경에서는 `test_mic.py`로 재확인이 필요하다.
- **`is_wakeup()`의 이중 threshold**: `model.predict(..., threshold=0.1)`과 `confidence > 0.5` 게이팅의 관계가 문서화돼 있지 않다.

## 알려진 제약 / TODO

- `package.xml`은 `<depend>hey_doopal_msg</depend>` 하나만 선언한다. `openwakeword`, `langchain`, `langchain-openai`, `openai`, `python-dotenv`, `scipy`, `pyaudio` 등은 전부 pip 의존성으로, `requirements.txt` 같은 별도 목록 없이 환경에 설치되어 있다는 전제로 동작한다.
- 시동어 이후 STT는 고정 5초 녹음(블로킹)이라, 사용자가 5초 안에 말을 끝내지 못하거나 더 짧게 말해도 항상 5초를 다 기다린다(무음 구간 자동 종료 같은 VAD 로직 없음).
- `main_loop`가 `rclpy.Timer`(0.1초 주기)로 구현돼 있어, `WAKEUP` 모드에서 매 0.1초마다 `is_wakeup()`이 마이크 버퍼를 폴링한다 — 이벤트 기반이 아니라 폴링 기반 설계다.
- LangChain 파이프라인은 매 발화마다 OpenAI API를 2번 호출한다(Whisper STT 1회 + gpt-4o 슬롯 추출 1회). 지연 시간과 비용이 발화 하나마다 API 왕복 2회에 좌우된다.
