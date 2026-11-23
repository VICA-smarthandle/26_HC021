#!/usr/bin/env bash
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
( sleep 5; s=$(du -sb "$out" 2>/dev/null | cut -f1); echo "[5 s 확인] $out ${s:-0} 바이트 — 0 이면 토픽·DOMAIN 확인" ) &
echo "끝낸 뒤: /odom 이 계속 나오는지 확인(10-11 녹화 종료 순간 EKF 멈춤 전례)"
exec ros2 bag record "${args[@]}" -o "$out" "${topics[@]}"
