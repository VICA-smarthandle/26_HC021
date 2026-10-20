#!/usr/bin/env bash
set -u

pids=$(pgrep -f 'python[0-9.]* .*-m src\.ros_' || true)
if [ -z "$pids" ]; then
    echo "음성 노드를 찾지 못했다 — 스택 기동 후에 실행할 것"
    exit 1
fi

echo "적용 전:"
ps -o pid,ni,comm,args -p $pids | cut -c1-90
renice -n 10 -p $pids > /dev/null
echo
echo "적용 후 (NI=10 이면 성공):"
ps -o pid,ni,comm,args -p $pids | cut -c1-90
