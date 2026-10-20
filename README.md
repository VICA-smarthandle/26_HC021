<div align="center">
  <img src="docs/images/banner.png" alt="VICA Smart Handle" width="720" />
  <h1>VICA</h1>
  <p><strong>Vision Companion · 손잡이를 잡고 함께 걷는 실내 전용 동행 로봇</strong></p>
  <p>'Smart Handle'과 LLM을 적용한 시각장애인 동행 AMR. 음성으로 목적지를 말하면, 손잡이를 잡은 채 목적지까지 함께 걷습니다.</p>
  <p>
    <a href="#overview">프로젝트 개요</a> ·
    <a href="#team">팀원 소개</a> ·
    <a href="#system">시스템 아키텍처</a> ·
    <a href="#video">작품 소개영상</a> ·
    <a href="#source">핵심 소스코드</a>
  </p>
  <p>
    <img src="https://img.shields.io/badge/2026-한이음%20드림업%20공모전-blue" alt="2026 한이음 드림업 공모전" />
    <img src="https://img.shields.io/badge/프로젝트%20번호-26__HC021-informational" alt="프로젝트 번호 26_HC021" />
    <img src="https://img.shields.io/badge/팀-VICA-success" alt="팀 VICA" />
  </p>
</div>

---

<a id="overview"></a>
## **1. 프로젝트 개요**

**1-1. 프로젝트 소개**
- 프로젝트 명 : VICA(Vision Companion), 'Smart Handle'과 LLM을 적용한 시각장애인 동행 AMR
- 프로젝트 정의 : 사용자가 목적지까지 쉽게 이동할 수 있도록 돕는 동행형 실내 안내 AMR

<!-- 비카 이미지 들어갈 곳(비카에 대한 설명까지 이미지로 설명할 수 있으면 좋음) -->

**1-2. 개발 배경 및 필요성**
- **안내 수단 부족:** 함께 걷는 안내 수단은 사실상 안내견뿐인데, 공급이 턱없이 부족합니다. 국내 시각장애인은 약 24.4만 명이지만 시각장애인 대 안내견 비율은 2,711 : 1입니다.
- **시각 중심 안내:** 병원·복지관·도서관 같은 시설의 안내는 점자표지판, 키오스크, 안내로봇처럼 눈으로 보고 따라가야 하는 방식이 대부분입니다.

> 💡 **VICA의 해결**<br>
> 음성, 손잡이 촉각 피드백, 자율주행, 객체 인식을 묶어서 화면을 보지 않고도 목적지까지 함께 동행할 수 있게 합니다.

**1-3. 프로젝트 특장점**

| 특장점 | 내용 | 자세히 |
|---|---|:---:|
| 자체 제작 하드웨어 | 프레임과 손잡이를 직접 설계·제작하고, 5개 모듈로 나눠 고장 나면 모듈째 교체 | [3-2](#arch-hw) |
| Smart Handle | 당기는 만큼 감속하고, 회전 방향을 손끝으로 미리 알리는 똑똑한 손잡이 | [3-3](#arch-handle) |
| 동행 전용 주행 (VCC) | 손잡이와 사람의 자리까지 계산해서 장애물을 부드럽게 피하고 미리 감속 | [3-4](#arch-nav) |
| 온디바이스 음성·LLM | STT, LLM, TTS를 모두 로봇 안에서 처리해서 대화 내용이 밖으로 나가지 않음 | [3-5](#arch-voice) |
| 관리자 앱 | 지도 작성부터 원격 주행, 배송, 진단까지 앱 하나로 관리 | [3-6](#arch-app) |

**1-4. 주요 기능**

<table>
<tr><th align="center" nowrap>순서</th><th nowrap>기능</th><th>내용</th></tr>
<tr><td align="center" nowrap>1</td><td nowrap>사용자 인식·접근</td><td>카메라와 객체인식모델(YOLOv11n커스텀학습)로 시각장애인 지팡이를 든 사람을 인식하면 다가가 안내를 제안합니다</td></tr>
<tr><td align="center" nowrap>2</td><td nowrap>스마트핸들</td><td>사용자가 사용 의사를 표현하면 로봇을 회전시켜 스마트핸들을 내밀고, 진동으로 위치를 알립니다</td></tr>
<tr><td align="center" nowrap>3</td><td nowrap>자연어로 목적지 지정</td><td>"화장실 가고 싶어", "배 아파"처럼 일상적인 표현을 추론해 등록된 장소와 연결하고, 목적지를 확인합니다</td></tr>
<tr><td align="center" nowrap>4</td><td nowrap>자율주행</td><td>관리자가 등록한 지도와 주행 레일을 따라 목적지까지 스스로 이동합니다. LiDAR·깊이 카메라·초음파로 장애물을 찾고, 직접 만든 동행 전용 컨트롤러(VCC)가 손잡이와 사람의 자리까지 계산해 부드럽게 피하고 코너 앞에서 미리 감속합니다</td></tr>
<tr><td align="center" nowrap>5</td><td nowrap>동행 주행</td><td>손잡이 당김을 감지해 사용자의 걸음속도에 맞춰 감속, 회전 방향을 디렉션 패들로 손끝으로 알리며 장애물을 우회주행합니다</td></tr>
<tr><td align="center" nowrap>상시</td><td nowrap>긴급 정지</td><td>대화 중이든 주행 중이든 긴급어를 상시 감시하고, LLM을 거치지 않고 바로 비상정지합니다</td></tr>
<tr><td align="center" nowrap>운영</td><td nowrap>관리자 앱</td><td>지도 작성, 목적지·홈·금지구역 등록, 원격 주행·배송, 비상정지, 로봇 상태 진단을 앱 하나로 관리합니다</td></tr>
</table>

**1-5. 기대 효과 및 활용 분야**
- 기대 효과 : 시각장애인이 누군가에게 안내를 부탁하지 않고 혼자 낯선 실내 시설을 이용할 수 있습니다. 안내 인력의 단순 동행 업무를 대신하고, 로컬 LLM 사용으로 클라우드 API 사용료와 통신료가 들지 않습니다.
- 활용 분야 : 요양·재활병원, 노인·장애인복지관, 공공도서관·관공서·공공청사 같은 복합 실내 시설 도입. 이후 고령자 혹은 길 찾기가 어려운 누구나 쓸 수 있는 시설 안내로봇, 짐 운반·물류 보조로 확장할 수 있습니다.

**1-6. 기술 스택**

<table>
<tr><th nowrap>분야</th><th>기술</th></tr>
<tr><td nowrap>연산·OS</td><td>Jetson Orin NX 16GB (reComputer J401) · Ubuntu 22.04 / JetPack 6.2 · CUDA</td></tr>
<tr><td nowrap>미들웨어·주행</td><td>ROS 2 Humble · Nav2 (Route Server, SmacPlannerLattice, 자체 컨트롤러 VCC) · Cartographer · AMCL · robot_localization(EKF)</td></tr>
<tr><td nowrap>인지</td><td>Intel RealSense D455 · YDLIDAR G2 · 초음파 · YOLO11s-seg(자체 데이터셋) + ByteTrack · TensorRT FP16 · Isaac ROS</td></tr>
<tr><td nowrap>음성·LLM</td><td>reSpeaker Mic Array v3.0 · openWakeWord(자체 학습 2종) · Qwen3-ASR 1.7B(transcribe.cpp) · KT Mi:dm 2.0 Mini(Ollama) · LangChain · Supertonic-3 · CosyVoice 3</td></tr>
<tr><td nowrap>구동·하드웨어</td><td>MDROBOT MD200T(CAN) + MDH100 인휠모터 ×2 · Smart Handle(로프릴·다회전 가변저항·서보·햅틱) · 24V LiFePO4</td></tr>
<tr><td nowrap>관리자 앱</td><td>Flutter(Dart) · rosbridge WebSocket · Android</td></tr>
<tr><td nowrap>시뮬레이션·도구</td><td>Isaac Sim 6.0 · Docker · Foxglove Studio · RViz2 · Git/GitHub</td></tr>
</table>

---

<a id="team"></a>
## **2. 팀원 소개(이름 기재X)**

| 🛠️ | 🧭 | 🎙️ | 📱 | 👨‍🏫 |
|:---:|:---:|:---:|:---:|:---:|
| **멘티1** | **멘티2** | **멘티3** | **멘티4** | **멘토** |
| • 하드웨어 설계·제작 <br> • 전장(전원·배선) | • 자율주행 <br> • 안전 제어 | • 음성 인터페이스 <br> • LLM | • 관리자 앱 | • 프로젝트 멘토 <br> • 기술 자문 |

---

<a id="system"></a>
## **3. 시스템 아키텍처**

| 3-1 | 3-2 | 3-3 | 3-4 | 3-5 | 3-6 | 3-7 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| [전체 구성도](#arch-overall) | [하드웨어](#arch-hw) | [스마트 핸들](#arch-handle) | [자율주행](#arch-nav) | [음성 서비스](#arch-voice) | [관리자 앱](#arch-app) | [객체 인식](#arch-vision) |

<a id="arch-overall"></a>
### 3-1. 시스템 구성도

- **명령 흐름** : 음성 → 판단 → 주행 → 안전·구동 순서로 전달
- **미션 매니저** : 모든 요청을 10가지 조건으로 확인 후 주행 허가
- **안전 관문** : 버튼·앱·음성 비상정지를 한곳에서 관리, 관리자 확인 후에만 재출발

<img width="900" alt="S/W 전체 구성도" src="docs/images/sw_overview.png" />
<img width="900" alt="서비스 흐름도" src="docs/images/service_flow.png" />

<details>
<summary>미션 매니저·안전 제어 상세 구성도</summary>

<img width="900" alt="임무 판단 층" src="docs/images/sw_mission.png" />
<img width="900" alt="안전·구동 층" src="docs/images/sw_safety.png" />

</details>

<a id="arch-hw"></a>
### 3-2. 하드웨어 구성

- **자체 제작** : 프레임·하우징·손잡이 직접 설계
- **모듈 구조** : 손잡이·센서·컴퓨터·배터리·구동 5개 모듈, 고장 시 모듈 단위 교체

| 구성 | 부품 |
|:---:|---|
| 연산 | Jetson Orin NX 16GB (reComputer J401) |
| 센서 | YDLIDAR G2, Intel RealSense D455, 초음파 센서(A22), reSpeaker Mic Array v3.0, 전류 센서(INA228) |
| 구동 | MDROBOT MD200T 모터 드라이버(CAN) + MDH100 인휠모터 ×2 |
| Smart Handle | 포텐셔미터 로프릴, 디렉션 패들, 햅틱 모터, 터치센서, LED 바, 센서·핸들 제어 MCU |
| 전원·안전 | 24V LiFePO4 배터리, DC 차단기, 24V→5V 컨버터, EMS 비상정지 스위치 |

<img width="900" alt="VICA 실물" src="docs/images/vica_photo.png" />
<img width="900" alt="H/W 구성도" src="docs/images/hw_overview.png" />

<details>
<summary>전체 결선도</summary>

<img width="900" alt="전체 결선도" src="docs/images/hw_wiring.png" />

</details>

<a id="arch-handle"></a>
### 3-3. 스마트 핸들

안내견 하네스에서 착안, 로봇이 사용자 걸음에 맞추는 손잡이

- **포텐셔미터 로프릴** : 당기는 만큼 감속, 끝까지 당기면 정지
- **디렉션 패들·LED 바** : 회전 전 방향 미리 알림
- **햅틱 모터** : 손잡이 위치·비상정지·도착을 진동으로 알림
- **터치센서** : 손을 놓으면 즉시 정지

<img width="900" alt="Smart Handle" src="docs/images/smart_handle.png" />

<a id="arch-nav"></a>
### 3-4. 자율주행

- **위치 추정** : 바퀴·IMU 센서 융합과 지도 매칭으로 현재 위치 계산
- **경로** : 관리자가 그린 주행 레일을 따라 이동, 막힌 구간만 우회
- **동행 전용 컨트롤러(VCC)** : 손잡이까지 포함한 크기로 계산, 장애물 부드럽게 회피, 코너 앞 미리 감속
- **장애물 인식** : 라이다·깊이 카메라·초음파가 높이별로 나눠 감지

<img width="900" alt="동행 전용 주행 알고리즘 VCC" src="docs/images/vcc.png" />

<details>
<summary>인지·위치·주행 상세 구성도</summary>

<img width="900" alt="인지·위치·주행 층" src="docs/images/sw_navigation.png" />

</details>

<a id="arch-voice"></a>
### 3-5. 음성 서비스

- **호출어·긴급어** : "비카야"·"멈춰" 상시 감지, 긴급어는 LLM 없이 즉시 정지
- **온디바이스 처리** : 음성 인식·LLM·음성 합성 모두 로봇 안에서 처리, 대화 내용 외부 전송 없음
- **안전한 LLM** : LLM은 목적지 제안만, 출발 결정은 미션 매니저
- **응답 속도** : 음성 인식 0.53초, LLM 판단 1.81초 (2026년 10월 실주행 중앙값)

<img width="900" alt="음성·대화 층" src="docs/images/sw_voice.png" />

<a id="arch-app"></a>
### 3-6. 관리자 앱

- **지도 설정** : 지도 작성, 목적지·홈·금지구역·주행 레일 지정
- **주행 관리** : 원격 주행·배송·비상정지, 부품별 상태 진단
- **알림** : 주행 실패·도움 요청 팝업, 배송 도착 문자
- **안전** : 앱 요청도 미션 매니저 확인 후 실행

<img width="900" alt="관리자 앱" src="docs/images/admin_app.png" />

<a id="arch-vision"></a>
### 3-7. 객체 인식 및 추적

- **자체 학습 모델** : 흰지팡이를 든 사람 데이터셋 직접 구축, YOLO11s-seg 학습
- **추적** : 같은 사람은 같은 ID로 추적, 깊이 카메라로 거리·위치 계산
- **접근** : 1.5~4 m 거리에 서 있는 사람에게 다가가 안내 제안
- **성능** : 로봇 시점 재학습으로 정밀도 0.513 → 0.998, 재현율 0.716 → 0.964, 초당 61.4회 인식

<img width="900" alt="사용자 인식·접근" src="docs/images/perception.png" />

---

<a id="video"></a>
## **4. 작품 소개영상**

> 🎬 **YouTube 작품 소개영상**
> VICA가 사용자와 손잡이로 연결된 채 함께 걷는 자율주행과, 호출어 "비카야" → LLM 목적지·의도 해석 → 음성 안내로 이어지는 음성 목적지 입력을 실제 로봇으로 시연한 영상입니다.

[![VICA 시연영상](docs/images/video_thumbnail.jpg)](https://www.youtube.com/watch?v=teIxBrbtmoQ)

▶️ 썸네일을 클릭하면 유튜브에서 영상이 재생됩니다.

---

<a id="source"></a>
## **5. 핵심 소스코드**

**저장소 구성**

| 저장소 | 내용 |
|---|---|
| [`vica_ros2_ws`](https://github.com/VICA-smarthandle/vica_ros2_ws) | ROS 2 워크스페이스. 자율주행, 미션 매니저, 안전 제어, 모터, Smart Handle, 사람 인식 |
| [`vica-voice-llm`](https://github.com/VICA-smarthandle/vica-voice-llm) | 음성 인터페이스. 호출어·긴급어 감지, STT, LLM 의도 해석, TTS |
| [`VICA_Supervisor`](https://github.com/VICA-smarthandle/VICA_Supervisor) | Flutter 관리자 앱 |

**코드 목록** (3장 시스템 아키텍처와 같은 순서)

| No. | 분류 | 기능 | 파일 | 언어 |
|:---:|---|---|---|:---:|
| 5-1 | 전체 시스템 | 미션 매니저 상태 관리 | [`mission_logic.py`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_mission_manager/vica_mission_manager/mission_logic.py) | Python |
| 5-2 | 전체 시스템 | 안전 관문 (Safety Gate) | [`safety_gate.py`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_safety/vica_safety/safety_gate.py) | Python |
| 5-3 | 하드웨어 | 모터 드라이버 CAN 제어 | [`motor_core.hpp`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/mdrobot_can_control_cpp/include/mdrobot_can_control_cpp/motor_core.hpp) | C++ |
| 5-4 | 스마트 핸들 | 가변저항 속도 조절 | [`motor_core.hpp`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/mdrobot_can_control_cpp/include/mdrobot_can_control_cpp/motor_core.hpp) | C++ |
| 5-5 | 자율주행 | ① 장애물을 피하는 차선 선택 | [`lanes.cpp`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/lanes.cpp) | C++ |
| 5-6 | 자율주행 | ② 코너·장애물 앞 미리 감속 | [`speed_profile.cpp`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/speed_profile.cpp) | C++ |
| 5-7 | 자율주행 | ③ 충돌 예측 정지 | [`vcc_core.cpp`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/vcc_core.cpp) | C++ |
| 5-8 | 음성 서비스 | STT → LLM → TTS | [`ros_node.py`](https://github.com/VICA-smarthandle/vica-voice-llm/blob/4f173e44b37111e3797209a455a16bf3aa842b7f/src/ros_node.py) | Python |
| 5-9 | 관리자 앱 | 주행·도움 요청 알림 | [`supervisor_provider.dart`](https://github.com/VICA-smarthandle/VICA_Supervisor/blob/748075575b60c9d0b1c157e4cc401a5772eca930/lib/providers/supervisor_provider.dart) | Dart |
| 5-10 | 객체 인식 | 사람 인식 및 추적 | [`person_detector_node.py`](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_perception/vica_perception/person_detector_node.py) | Python |

<br>

### 5-1. 전체 시스템 ① 미션 매니저 상태 관리

📄 [`mission_logic.py` L1333–L1371](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_mission_manager/vica_mission_manager/mission_logic.py#L1333-L1371)

음성, 관리자 앱, 사람 인식에서 오는 요청은 모두 미션 매니저를 거칩니다. 로봇의 상태(18개)에 따라 같은 말에도 다르게 반응하고, 목적지로 출발하기 전에는 `check_gate()` 로 10가지 조건(목적지 확정, 사용자 확인, 비상정지, 좌표, Nav2 준비 등)을 확인합니다.

- 로봇을 직접 움직이지 않고 할 일 목록만 돌려주는 구조라서, ROS 없이도 18개 상태 × 11개 요청 = 198가지 경우를 테스트로 확인했습니다.

<img width="900" alt="mission_logic.py L1333-L1371" src="docs/images/code/code_5-1.png" />

<br>

### 5-2. 전체 시스템 ② 안전 관문 (Safety Gate)

📄 [`safety_gate.py` L62–L88](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_safety/vica_safety/safety_gate.py#L62-L88)

모든 주행 명령은 이 관문을 지나야 모터로 갑니다. 버튼, 앱, 음성("멈춰") 비상정지는 하나로 모이고, 비상정지가 풀려도 관리자가 앱에서 다시 시작을 눌러야만 움직입니다.

<img width="900" alt="safety_gate.py L62-L88" src="docs/images/code/code_5-2.png" />

<br>

### 5-3. 하드웨어 · 모터 드라이버 CAN 제어

📄 [`motor_core.hpp` L428–L451](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/mdrobot_can_control_cpp/include/mdrobot_can_control_cpp/motor_core.hpp#L428-L451) · [`motor_core.hpp` L292–L312](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/mdrobot_can_control_cpp/include/mdrobot_can_control_cpp/motor_core.hpp#L292-L312)

안전 관문을 통과한 속도 명령(`/cmd_vel_safe`)을 좌우 바퀴의 rpm으로 바꿔 MDROBOT MD200T 드라이버에 CAN으로 보냅니다. 두 바퀴 속도는 8바이트 프레임 하나에 같이 담습니다. 최고 속도는 그 전에 손잡이 값으로 먼저 제한됩니다(5-4).

- Python으로 먼저 만든 뒤 C++로 옮겼습니다. 옮기고 나서 같은 입력 36만 개에서 두 버전이 보내는 CAN 프레임이 똑같은지 비교했습니다.

<img width="900" alt="motor_core.hpp L428-L451" src="docs/images/code/code_5-3a.png" />
<img width="900" alt="motor_core.hpp L292-L312" src="docs/images/code/code_5-3b.png" />

<br>

### 5-4. 스마트 핸들 · 가변저항 속도 조절

📄 [`motor_core.hpp` L106–L130](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/mdrobot_can_control_cpp/include/mdrobot_can_control_cpp/motor_core.hpp#L106-L130) · [`motor_core.hpp` L406–L426](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/mdrobot_can_control_cpp/include/mdrobot_can_control_cpp/motor_core.hpp#L406-L426)

손잡이 줄은 로프릴 안의 다회전 가변저항과 연결되어 있습니다. 줄을 당기면 저항값이 바뀌고, 모터 드라이버가 이 값을 0~100 %로 보내 줍니다. 이 비율만큼 최고 속도를 낮추기 때문에 사용자가 당기면 로봇이 느려지고, 끝까지 당기면 멈춥니다.

- 처음에는 0~100을 그대로 썼는데, 실제 줄은 55~98 구간에서만 움직여서 당겨도 속도가 거의 바뀌지 않았습니다. 이 구간을 0~100 %로 다시 늘려서 해결했습니다.

<img width="900" alt="motor_core.hpp L106-L130" src="docs/images/code/code_5-4a.png" />
<img width="900" alt="motor_core.hpp L406-L426" src="docs/images/code/code_5-4b.png" />

<br>

### 5-5. 자율주행 ① 장애물을 피하는 차선 선택

📄 [`lanes.cpp` L50–L70](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/lanes.cpp#L50-L70) · [`lanes.cpp` L92–L125](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/lanes.cpp#L92-L125)

VICA는 손잡이로 사람과 연결되어 있어서, 장애물 앞에서 급하게 꺾으면 사람이 그대로 끌려갑니다. 그래서 직접 만든 컨트롤러(VCC)는 주행 레일을 좌우로 0.1 m씩 옮긴 13개 차선(±0.6 m)을 만들고, 매 주기 점수가 가장 낮은 차선으로 부드럽게 옮겨 갑니다.

- 점수는 `10 × (장애물과 20 cm 거리에 모자란 만큼) + 1 × (레일에서 벗어난 거리) + 0.5 × (차선을 바꾸는 폭)` 입니다.
- 차선을 너무 자주 바꾸면 손잡이가 좌우로 흔들려서, 막혔을 때가 아니면 바로 바꾸지 않습니다.

<img width="900" alt="lanes.cpp L50-L70" src="docs/images/code/code_5-5a.png" />
<img width="900" alt="lanes.cpp L92-L125" src="docs/images/code/code_5-5b.png" />

<br>

### 5-6. 자율주행 ② 코너·장애물 앞 미리 감속

📄 [`speed_profile.cpp` L9–L45](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/speed_profile.cpp#L9-L45)

코너에 들어간 뒤에 속도를 줄이면 뒤따르는 사람이 덜컹거림을 그대로 느낍니다. 그래서 앞 1.5 m 경로를 미리 보고, 코너나 장애물이 있으면 닿기 전에 속도를 줄입니다.

- 회전 반지름 0.8 m보다 급한 코너, 장애물과 0.35 m보다 가까운 구간에서 속도가 줄어듭니다.
- `sqrt(v² + 2·a·d)` 는 "지금부터 감속하면 코너에서 목표 속도가 되는 최대 속도"입니다.

<img width="900" alt="speed_profile.cpp L9-L45" src="docs/images/code/code_5-6.png" />

<br>

### 5-7. 자율주행 ③ 충돌 예측 정지

📄 [`vcc_core.cpp` L130–L144](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_vcc_controller/src/core/vcc_core.cpp#L130-L144)

지금 속도로 멈추는 데 필요한 거리만큼 앞 경로에 로봇을 미리 놓아 보고, 손잡이를 포함한 몸이 장애물에 닿으면 바로 멈춥니다. `clearance()` 는 장애물 지도(costmap)와 초음파 값을 함께 봅니다.

<img width="900" alt="vcc_core.cpp L130-L144" src="docs/images/code/code_5-7.png" />

<br>

### 5-8. 음성 서비스 · STT → LLM → TTS

📄 [`ros_node.py` L211–L246](https://github.com/VICA-smarthandle/vica-voice-llm/blob/4f173e44b37111e3797209a455a16bf3aa842b7f/src/ros_node.py#L211-L246) · [`ros_node.py` L272–L284](https://github.com/VICA-smarthandle/vica-voice-llm/blob/4f173e44b37111e3797209a455a16bf3aa842b7f/src/ros_node.py#L272-L284)

"비카야"라고 부른 뒤 한 말은 Qwen3-ASR이 글자로 바꿔 이 함수로 들어옵니다. 긴급어가 있으면 바로 긴급 처리하고, 나머지는 로컬 LLM(Mi:dm 2.0 Mini)이 의도와 목적지를 고릅니다. LLM의 결과는 로봇을 직접 움직이지 않고, 미션 매니저에 요청으로만 전달됩니다.

- STT, LLM, TTS가 모두 로봇 안에서 돌아가서 목소리와 대화 내용이 외부로 나가지 않고, 인터넷이 끊겨도 동작합니다.
- "그래", "아니" 같은 짧은 대답은 LLM 없이 바로 처리해서 기다리는 시간이 없습니다.

<img width="900" alt="ros_node.py L211-L246" src="docs/images/code/code_5-8a.png" />
<img width="900" alt="ros_node.py L272-L284" src="docs/images/code/code_5-8b.png" />

<br>

### 5-9. 관리자 앱 · 주행·도움 요청 알림

📄 [`supervisor_provider.dart` L1677–L1684](https://github.com/VICA-smarthandle/VICA_Supervisor/blob/748075575b60c9d0b1c157e4cc401a5772eca930/lib/providers/supervisor_provider.dart#L1677-L1684) · [`supervisor_provider.dart` L1714–L1747](https://github.com/VICA-smarthandle/VICA_Supervisor/blob/748075575b60c9d0b1c157e4cc401a5772eca930/lib/providers/supervisor_provider.dart#L1714-L1747)

로봇이 보내는 주행 이벤트(출발, 도착, 실패, 취소, 도움 요청)를 받아 관리자에게 알립니다. 사용자의 도움 요청은 주행 알림과 섞이지 않게 따로 띄우고, 배송이 끝나면 문자로도 알려 줍니다.

- 알림이 너무 많으면 관리자가 읽지 않고 닫게 됩니다. 그래서 성공은 기록만 남기고, 같은 사건은 한 번만 띄웁니다.

<img width="900" alt="supervisor_provider.dart L1677-L1684" src="docs/images/code/code_5-9a.png" />
<img width="900" alt="supervisor_provider.dart L1714-L1747" src="docs/images/code/code_5-9b.png" />

<br>

### 5-10. 객체 인식 및 추적 · 시각장애인 찾기

📄 [`person_detector_node.py` L130–L177](https://github.com/VICA-smarthandle/vica_ros2_ws/blob/1c9d27539b0752226bd1966067c70c78e60d43f4/src/vica_perception/vica_perception/person_detector_node.py#L130-L177)

흰지팡이를 든 사람을 직접 모은 데이터로 학습한 YOLO11s-seg 모델을 TensorRT로 돌립니다. 같은 사람은 계속 같은 ID로 따라가고, 1초 넘게 안정적으로 보이면서 3초 동안 1.5~4 m 거리에 서 있으면 미션 매니저에 다가가도 되는지 요청합니다.

- 주행 중에는 추론을 멈춰서 Jetson의 CPU를 주행에 양보합니다.

<img width="900" alt="person_detector_node.py L130-L177" src="docs/images/code/code_5-10.png" />

<br>

### 📁 저장소 구조

코드는 역할별로 저장소 3개에 나뉘어 있고, 이 저장소(26_HC021)는 소개·문서·개발 기록을 모아 둔 곳입니다. ⭐ 표시는 위 핵심 소스코드가 있는 위치입니다.

```
VICA-smarthandle (GitHub 조직)
│
├── 26_HC021/                           ← 지금 이 저장소: 소개·문서·개발 기록
│   ├── README.md
│   ├── docs/                           설계 문서, README 그림, 팀 작업 안내
│   ├── devlog/                         날짜별 개발 기록
│   ├── guideline/                      팀 개발 규칙
│   ├── scripts/                        실행·측정 도구
│   └── workspace.repos                 코드 저장소 3개를 한 번에 받는 목록
│
├── vica_ros2_ws/                       ROS 2 워크스페이스 (자율주행·미션·안전·하드웨어)
│   └── src/
│       ├── vica_mission_manager/       미션 매니저 (요청 확인·상태 관리)  ⭐ 5-1
│       ├── vica_safety/                안전 관문·비상정지  ⭐ 5-2
│       ├── mdrobot_can_control_cpp/    모터 드라이버 CAN·손잡이 속도 조절 (C++)  ⭐ 5-3 5-4
│       ├── vica_vcc_controller/        동행 전용 컨트롤러 VCC (C++)  ⭐ 5-5 ~ 5-7
│       ├── vica_perception/            사람 인식·추적 (YOLO)  ⭐ 5-10
│       ├── vica_nav2/                  Nav2 설정·행동 트리·주행 레일
│       ├── vica_localization/          EKF 위치 추정
│       ├── vica_cartographer/          지도 작성
│       ├── vica_user_guidance(_cpp)/   Smart Handle 방향 예고·진동·LED, 펌웨어
│       ├── vica_sensor_adapters(_cpp)/ IMU·초음파 변환, 넘어짐 감지
│       ├── vica_destination_manager/   지도별 목적지 저장·조회
│       ├── vica_system_monitor/        로봇 상태 감시 (앱 진단 화면)
│       ├── vica_interfaces/            공용 메시지·서비스
│       └── ...                         로봇 모델, 바퀴 오도메트리, 라이다 등
│
├── vica-voice-llm/                     음성 서비스
│   ├── src/                            호출어·긴급어·STT·LLM·TTS 노드  ⭐ 5-8
│   ├── models/                         호출어·긴급어 감지 모델 (자체 학습)
│   ├── ollama/                         로컬 LLM 설정
│   ├── assets/                         미리 만든 안내 음성
│   └── tests/                          자동 테스트
│
└── VICA_Supervisor/                    관리자 앱 (Flutter)
    ├── lib/
    │   ├── providers/                  상태·알림 처리  ⭐ 5-9
    │   ├── screens/ · widgets/         화면
    │   ├── ros/                        rosbridge 연결
    │   └── models/ · services/         데이터 형식, 배송 문자 등
    └── ros2/                           앱용 ROS 2 보조 도구
```
