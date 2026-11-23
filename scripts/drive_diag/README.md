# drive_diag — 주행 진단 도구

녹화된 rosbag 만 읽는다(주행 중인 ROS 에 붙지 않음, 노드 추가 없음). ROS 2 환경을 먼저 불러온다:
`source /opt/ros/humble/setup.bash`

| 도구 | 하는 일 |
| --- | --- |
| `serve.py` | **x·y·θ 오차 뷰어**. 브라우저에서 bag·지도를 고르면 ①②를 계산·캐시하고 그래프·지도로 보여 준다 |
| `analyze_loc.py` | 뷰어가 쓰는 계산을 명령줄로(요약 출력). `analyze_loc.py <bag> <지도.yaml>` |
| `laser_yaw.py` | 라이다 장착각 측정. `wall`(정지 벽 맞춤) · `drive`(전진·후진) |
| `laser_yaw_record.sh` | 장착각 측정용 녹화(`wall`·`drive`) |
| `metrics.py` | 흔들림·치우침 판정 계산(점검 1~5) |
| `record.sh` | 주행 진단 녹화(Nav2·teleop). `record.sh [이름]` → `MMDD_HHMMSS_<지도>_<이름>`, 가벼운 토픽만·best_effort |
| `compare.py` | **주행 bag 비교**. 직진 중 AMCL 보정량(cm/m·°/m·1회 크기)·① 위치 오차·지도 정합·흔들림을 표 하나로, 첫 bag vs 나머지 판정 |
| `map_scale.py` | **지도 길이 보정**. `--to 기준지도` 로 배율 추정(아핀 정합) → `--scale` 시험본 → `--apply`(원본 백업). 레일·금지구역 좌표도 같은 배율 |
| `wheel_check.py` | **바퀴 점검**(CAN 받기만). 직진 명령 rpm ↔ 엔코더(C5) 비율, 한 바퀴 틱 vs 설정값. `live`·`file` |

## x·y·θ 오차 뷰어

```bash
python3 scripts/drive_diag/serve.py          # http://127.0.0.1:8765
# 노트북에서 보기: ssh -L 8765:127.0.0.1:8765 <젯슨>  → 노트북 브라우저 http://127.0.0.1:8765
```

- **① 위치추정 오차 = AMCL 자세 − 지도 대조 자세.** 스캔마다(2 Hz) AMCL 자세 주변 ±25 cm·±6° 에서 스캔이
  지도에 가장 잘 맞는 자세를 찾는다(잣대 = `vica_nav2/pose_score.py` 의 likelihood field·sigma_hit 0.2,
  거리 필드만 칸 사이 선형 보간). 지도 정합 점수가 낮은 구간(주황 띠, 최고 점수 < 65 %)은 지도와 실제가 달라 믿기 어렵다.
  범위 끝에 걸린 장면은 빨간 ×.
- **② 오도메트리 오차 = 믿은 이동 − 라이다 이동(구간 시작 기준).** 라이다 이동 = 0.25 m·3° 마다 스캔-스캔
  점-선 ICP 를 이어 붙이고 TF(base→laser)로 차체 이동으로 바꾼 것. EKF(바퀴 거리 + 자이로 방향)와 바퀴만을 비교.
  옆·θ 는 믿을 만하고, 앞(거리)은 ICP 거리 자체가 2~3.5 % 길게 나와 참고만.
- 필요한 녹화 토픽: `/scan /tf /tf_static /odom` (+ `/wheel/odom /imu/base_link /cmd_vel_safe`). ① 은 AMCL 이
  위치를 낸 bag(map→odom 있음)에서만, 지도는 그 장소의 것을 골라야 한다.
- 개요를 드래그하면 구간, 그래프를 누르면 그 시각의 자세·스캔이 지도에 그려진다. 주소(`?bag=…&map=…&sel=a,b&t=c`)로 같은 화면을 공유.
- 캐시: `~/vica_data/drive_diag_cache/`. 지도·bag 이 바뀌면 자동으로 새로 계산(`다시 계산` 으로 강제).

부호: 옆·방향 + = 왼쪽(반시계), 앞·옆은 로봇 방향 기준.

## 바퀴 점검 (wheel_check.py)

rosbag 이 아니라 can1 을 직접 받는다(송신 없음, ROS 불필요). 모터 노드가 명령(0x001 CF)을 보낼 때만 드라이버가
위치(0x701 C5)를 돌려주므로 모터 노드는 떠 있어야 한다. 기본 한 바퀴 틱·반지름은 `encoder.yaml`(인자 `--tpr`·`--radius` 로 덮어씀).

```bash
# 로봇 정지 상태에서 켜고 teleop 으로 직진(일정 속도 2 s 이상), 멈춘 뒤 Ctrl-C → 요약
python3 scripts/drive_diag/wheel_check.py live --save ~/vica_data/wheel_check/$(date +%m%d_%H%M%S).log
# 다시 보기 + 타이어 마크로 센 바퀴 수·줄자 거리(주행 덩어리 1 기준, --run 으로 바꿈)
python3 scripts/drive_diag/wheel_check.py file <로그> --revs 9.8 --tape-m 4.00
```

- 비율 = 측정 rpm(틱/s ÷ tpr × 60) ÷ 명령 rpm. 속도 기준 한 바퀴 틱 = 틱/s × 60 ÷ 명령 rpm.
- 판정 [잠정]: 비율 0.98~1.02 = 같음, 실측 한 바퀴 틱이 설정값 ±1 % = 맞음.
- 가변저항: 드라이버 `701#F1` d[6] 원값과 모터 노드 상한(knob_min 55~max 98 → 0~100 %, × max_linear 1.0)을 같이 보인다. 비율은 실제로 나간 CAN 명령 대비라 소프트웨어 상한은 안 들어간다. 같은 속도로 가변저항 위치만 바꿔 달리면 '위치별 직진 비율'로 드라이버가 가변저항을 쓰는지 가른다(차 < 0.01 = 안 씀). 0.5 m/s 를 안 자르는 최저 원값 77 %.
- `candump -L can1,001:7FF,701:7FF` 기록도 file 모드로 읽는다. 2026-10-12 run92 기록: 비율 R 0.991·L 0.985 → 한 바퀴 60.65·60.30 틱(설정 61.2).

## 시험

```bash
python3 -m pytest scripts/tests/test_drive_diag_*.py
```
