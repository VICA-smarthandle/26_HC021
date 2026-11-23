#!/usr/bin/env bash
set -euo pipefail

name=${1:-nav}
topics=(/scan /tf /tf_static /odom /wheel/odom /imu/base_link /amcl_pose /initialpose /cmd_vel_req /cmd_vel_safe /plan)

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ws="$here/../../vica_ros2_ws/install/setup.bash"
maps="$here/../../vica_ros2_ws/maps"
set +u
source /opt/ros/humble/setup.bash
[ -f "$ws" ] && source "$ws"
set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0} \
       RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

map=$(tr -d '[:space:]' < "$maps/CURRENT_MAP" 2>/dev/null || echo nomap)
root=${VICA_BAG_ROOT:-$HOME/vica_data/bags}
out="$root/$(date +%m%d_%H%M%S)_${map}_${name}"
qos=${VICA_BAG_QOS:-$HOME/vica_data/dds/bag_qos_best_effort.yaml}
args=(--include-hidden-topics)
[ -f "$qos" ] && args+=(--qos-profile-overrides-path "$qos")

echo "녹화: $out"
echo "지도: $map (CURRENT_MAP — Nav2 가 실제로 이 지도로 떠 있는지 rviz 로 확인)"
echo "토픽: ${topics[*]}  (DOMAIN $ROS_DOMAIN_ID · $RMW_IMPLEMENTATION)"
echo "순서: 초기 위치(2D Pose Estimate) 먼저 → 녹화 → 정지 5 s → 주행 → 정지 5 s → Ctrl-C"
( sleep 5; s=$(du -sb "$out" 2>/dev/null | cut -f1); echo "[5 s 확인] $out ${s:-0} 바이트 — 0 이면 토픽·DOMAIN 확인" ) &
echo "끝낸 뒤: /odom 이 계속 나오는지 확인(10-11 녹화 종료 순간 EKF 멈춤 전례)"
exec ros2 bag record "${args[@]}" -o "$out" "${topics[@]}"
