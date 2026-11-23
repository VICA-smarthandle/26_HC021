#!/usr/bin/env bash
# 라이다 장착각 측정 녹화. 끝낼 때는 Ctrl-C (SIGINT).
#
#   scripts/drive_diag/laser_yaw_record.sh wall    # 방법 1: 차체를 벽과 평행하게 세운 채 30 s
#   scripts/drive_diag/laser_yaw_record.sh drive   # 방법 2: teleop w=0 4 m 전진 ×3 · 같은 선 후진 ×3
#
# 저장 위치: ${VICA_BAG_ROOT:-~/vica_data/bags}/MMDD_HHMMSS_laser_yaw_<mode>
# QoS: ${VICA_BAG_QOS:-~/vica_data/dds/bag_qos_best_effort.yaml} 가 있으면 쓴다(빠른 토픽 best_effort —
#      10-16 녹화기 UDP 넘침·재전송 폭주 재발 방지). 없으면 기본 QoS.
# 분석: python3 scripts/drive_diag/laser_yaw.py wall|drive <저장 폴더>
#
# 녹화는 바퀴를 움직이지 않는다. 방법 2 의 주행은 사람이 teleop 칸에서 한다(i 전진 · , 후진 — w 가 정확히 0).
set -euo pipefail

mode=${1:-}
case "$mode" in
  wall)  topics=(/scan /tf_static) ;;
  drive) topics=(/scan /tf /tf_static /odom /wheel/odom /imu/base_link /cmd_vel_req /cmd_vel_safe) ;;
  *) echo "쓰기: $0 wall|drive" >&2; exit 2 ;;
esac

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ws="$here/../../vica_ros2_ws/install/setup.bash"
set +u
source /opt/ros/humble/setup.bash
[ -f "$ws" ] && source "$ws"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0} \
       RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

root=${VICA_BAG_ROOT:-$HOME/vica_data/bags}
out="$root/$(date +%m%d_%H%M%S)_laser_yaw_${mode}"
qos=${VICA_BAG_QOS:-$HOME/vica_data/dds/bag_qos_best_effort.yaml}
args=(--include-hidden-topics)
[ -f "$qos" ] && args+=(--qos-profile-overrides-path "$qos")

echo "녹화: $out"
echo "토픽: ${topics[*]}  (DOMAIN $ROS_DOMAIN_ID · $RMW_IMPLEMENTATION)"
if [ "$mode" = wall ]; then
  echo "확인: 차체 앞·뒤 두 점(간격 ≥ 0.5 m) 벽까지 줄자 차이 ≤ 2 mm · 벽 2 m 이상 · 유리/거울 아님 · 30 s 뒤 Ctrl-C"
else
  echo "확인: 바퀴·주변 안전 · teleop w=0 키(i 전진 · , 후진)로 0.3 m/s 4 m 전진 ×3, 같은 선 후진 ×3 · 끝나면 Ctrl-C"
fi
# 실제로 쓰이는지 5 s 뒤 크기 확인(녹화 중 bag 은 열지 않고 파일 크기만 본다)
( sleep 5; s=$(du -sb "$out" 2>/dev/null | cut -f1); echo "[5 s 확인] $out ${s:-0} 바이트 — 0 이면 토픽·DOMAIN 확인" ) &
echo "끝낸 뒤: /odom 이 계속 나오는지 확인(10-11 녹화 종료 순간 EKF 멈춤 전례)"
exec ros2 bag record "${args[@]}" -o "$out" "${topics[@]}"
