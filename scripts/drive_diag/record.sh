#!/usr/bin/env bash
# 주행 진단 녹화(Nav2·teleop 공통). 끝낼 때는 Ctrl-C (SIGINT).
#
#   scripts/drive_diag/record.sh [이름]      # 이름 기본 nav — 폴더 MMDD_HHMMSS_<지도>_<이름>
#
# 담는 것: 위치추정(AMCL 보정·지도 대조) · 오도메트리 · 흔들림 분석에 쓰는 토픽만(영상·costmap 없음 — 가볍게).
# /plan 은 플래너 경로 — 흔들림이 경로 모양(플래너)인지 따라가기(컨트롤러)인지 가르는 데 쓴다(11-02 추가).
# QoS: ${VICA_BAG_QOS:-~/vica_data/dds/bag_qos_best_effort.yaml} 가 있으면 쓴다(빠른 토픽 best_effort —
#      10-16 녹화기 UDP 넘침·재전송 폭주 재발 방지).
# 분석: python3 scripts/drive_diag/compare.py <저장 폴더> [비교할 폴더 …]   (지도는 bag 이름의 지도로 찾음)
#
# 녹화는 바퀴를 움직이지 않는다. 주행은 사람이 Nav2·teleop 으로 한다.
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
# 실제로 쓰이는지 5 s 뒤 크기 확인(녹화 중 bag 은 열지 않고 파일 크기만 본다)
( sleep 5; s=$(du -sb "$out" 2>/dev/null | cut -f1); echo "[5 s 확인] $out ${s:-0} 바이트 — 0 이면 토픽·DOMAIN 확인" ) &
echo "끝낸 뒤: /odom 이 계속 나오는지 확인(10-11 녹화 종료 순간 EKF 멈춤 전례)"
exec ros2 bag record "${args[@]}" -o "$out" "${topics[@]}"
