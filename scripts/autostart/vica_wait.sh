#!/bin/bash
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

tries=$(( limit * 5 ))
while :; do
  check && exit 0
  [ "$tries" -le 0 ] && break
  tries=$(( tries - 1 ))
  sleep 0.2
done

echo "[vica-wait] ${limit}초 안에 준비되지 않았다: $kind $what" >&2
exit 1
