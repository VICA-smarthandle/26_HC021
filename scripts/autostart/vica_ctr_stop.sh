#!/bin/bash
# 유닛 묶음(cgroup) 밖에서 도는 프로세스를 내린다 (ExecStop · ExecStopPost).
#
#   vica_ctr_stop.sh <명령줄 패턴> [초=10]
#
# `docker exec` 로 띄운 프로세스는 컨테이너 쪽 소속이라 systemd 의 정지 신호가 닿지 않는다.
# 컨테이너는 호스트와 같은 uid 로 돌고 호스트 프로세스 표에 보이므로, 호스트에서 명령줄로
# 찾아 SIGINT → (시한) → SIGTERM → SIGKILL 로 올린다. dockerd 가 멈춰 있어도 동작한다.
pattern="${1:-}"
limit="${2:-10}"
if [ -z "$pattern" ]; then
  sed -n '2,4p' "$0" >&2
  exit 2
fi

# 패턴이 인자로 들어오므로 이 스크립트(와 그 하위 셸)의 명령줄도 패턴에 걸린다. 빼고 센다.
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
