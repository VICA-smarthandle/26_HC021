#!/bin/bash
pattern="${1:-}"
limit="${2:-10}"
if [ -z "$pattern" ]; then
  sed -n '2,4p' "$0" >&2
  exit 2
fi

targets() {
  local pid
  for pid in $(pgrep -f -- "$pattern"); do
    [ "$pid" = "$$" ] && continue
    [ "$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')" = "$$" ] && continue
    echo "$pid"
  done
}

send() {
  local pids
  pids=$(targets)
  [ -n "$pids" ] && kill "-$1" $pids 2>/dev/null
}

gone_within() {
  local tries=$(( $1 * 5 ))
  while [ "$tries" -gt 0 ]; do
    [ -z "$(targets)" ] && return 0
    tries=$(( tries - 1 ))
    sleep 0.2
  done
  [ -z "$(targets)" ]
}

[ -z "$(targets)" ] && exit 0

send INT
gone_within "$limit" && exit 0
echo "[vica-ctr-stop] ${limit}초 안에 안 내려가 SIGTERM: $pattern" >&2
send TERM
gone_within 3 && exit 0
echo "[vica-ctr-stop] SIGKILL: $pattern" >&2
send KILL
gone_within 2 && exit 0
echo "[vica-ctr-stop] 남은 pid: $(targets | tr '\n' ' ')" >&2
exit 1
