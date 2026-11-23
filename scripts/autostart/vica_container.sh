#!/bin/bash
NAME=vica_rs_container
IMAGE=vica_isaac_ros_realsense_2563:latest
WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"

trap 'exit 0' INT TERM

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "[vica-container] 이미 떠 있다 — 붙기만 한다."
elif docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker start "$NAME" >/dev/null || {
    echo "[vica-container] docker start 실패." >&2
    exit 1
  }
  echo "[vica-container] 멈춰 있던 컨테이너를 시작했다."
else
  docker run -d -t -i \
    --name "$NAME" \
    --net=host \
    --ipc=host \
    --privileged \
    --runtime nvidia \
    --user 1000:1000 \
    --group-add 44 \
    --group-add 111 \
    -e HOME="$HOME" \
    -e ROS_LOG_DIR="$HOME/.ros/log" \
    -e ROS_DOMAIN_ID=7 \
    -e ROS_LOCALHOST_ONLY=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -v "$WS":/workspaces/isaac_ros-dev \
    "$IMAGE" \
    bash -c "cd /workspaces/isaac_ros-dev && \
      chmod a+rw /dev/bus/usb/*/* 2>/dev/null || true; \
      chmod a+rw /dev/video* 2>/dev/null || true; \
      chmod a+rw /dev/hidraw* 2>/dev/null || true; \
      exec bash" >/dev/null || {
    echo "[vica-container] docker run 실패." >&2
    exit 1
  }
  echo "[vica-container] 컨테이너를 새로 만들었다."
fi

docker wait "$NAME" >/dev/null &
wait $!
echo "[vica-container] 컨테이너가 끝났다." >&2
exit 70
