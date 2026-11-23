#!/bin/bash
set -u

IFACE="${1:-can1}"
BITRATE=500000

detail=$(ip -details link show "$IFACE" 2>&1) || {
  echo "[vica-can] $IFACE 를 읽지 못했다: $detail" >&2
  exit 1
}

if grep -qE '<[^>]*\bUP\b[^>]*>' <<< "$detail"; then
  now=$(grep -oE 'bitrate [0-9]+' <<< "$detail" | head -1 | awk '{print $2}')
  if [ "$now" = "$BITRATE" ]; then
    echo "[vica-can] $IFACE 는 이미 UP · bitrate $BITRATE — 그대로 둔다."
    exit 0
  fi
  echo "[vica-can] $IFACE 가 UP 인데 bitrate 가 ${now:-알 수 없음} 이다(기대 $BITRATE)." >&2
  echo "           링크를 내리지 않으므로 여기서 멈춘다. 사람이 확인한다." >&2
  exit 2
fi

ip link set "$IFACE" type can bitrate "$BITRATE" berr-reporting on restart-ms 100 || {
  echo "[vica-can] $IFACE bitrate 설정 실패 — 올리지 않는다." >&2
  exit 3
}
ip link set "$IFACE" up || {
  echo "[vica-can] $IFACE up 실패." >&2
  exit 4
}
echo "[vica-can] $IFACE UP · bitrate $BITRATE · restart-ms 100"
