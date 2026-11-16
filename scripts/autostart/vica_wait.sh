#!/bin/bash
# 유닛을 띄우기 전에 선행 조건을 시한 안에서 기다린다 (ExecStartPre).
#
#   vica_wait.sh path <경로> [초]           파일·장치가 생길 때까지
#   vica_wait.sh proc <comm> [초]           그 이름의 프로세스가 뜰 때까지 (pgrep -x, 15자)
#   vica_wait.sh canup <인터페이스> [초]    링크가 UP 일 때까지
#   vica_wait.sh container <이름> [초]      컨테이너가 running 일 때까지
#
# 시한(기본 60초)을 넘기면 1, 사용법이 틀리면 2. 터미네이터에서는 사람이 순서를 지켰다 —
# 유닛에서는 이 스크립트가 그 자리를 맡는다.
kind="${1:-}"
what="${2:-}"
limit="${3:-60}"

check() {
  case "$kind" in
    path) [ -e "$what" ] ;;
    proc) pgrep -x "$what" >/dev/null 2>&1 ;;
    canup) ip link show "$what" 2>/dev/null | grep -qE '<[^>]*\bUP\b[^>]*>' ;;
    container) [ "$(docker inspect -f '{{.State.Running}}' "$what" 2>/dev/null)" = "true" ] ;;
  esac
}

case "$kind" in
  path|proc|canup|container) [ -n "$what" ] || kind="" ;;
  *) kind="" ;;
esac
if [ -z "$kind" ]; then
  sed -n '2,8p' "$0" >&2
  exit 2
fi

# 0.2초 간격. SECONDS 는 정수 초라 1초짜리 시한을 0.1초 만에 넘길 수 있어 횟수로 센다.
tries=$(( limit * 5 ))
while :; do
  check && exit 0
  [ "$tries" -le 0 ] && break
  tries=$(( tries - 1 ))
  sleep 0.2
done

echo "[vica-wait] ${limit}초 안에 준비되지 않았다: $kind $what" >&2
exit 1
