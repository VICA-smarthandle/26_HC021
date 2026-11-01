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

## 시험

```bash
python3 -m pytest scripts/tests/test_drive_diag_*.py
```
