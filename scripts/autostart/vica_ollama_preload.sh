#!/bin/bash
model="${1:-midm2-mini}"
limit="${2:-180}"
host="${OLLAMA_HOST:-http://localhost:11434}"
start=$(date +%s)

for _ in $(seq 1 30); do
    curl -sf --max-time 2 "$host/api/tags" >/dev/null && break
    sleep 1
done

pid=$(pgrep -xo ollama)
if [ -n "$pid" ] && ! tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx 'OLLAMA_GO_TEMPLATE=1'; then
    echo "[Ollama] 경고: 떠 있는 Ollama(pid $pid)에 OLLAMA_GO_TEMPLATE=1 이 없습니다 — 끄고 음성을 다시 띄우세요"
fi

if curl -sf --max-time "$limit" "$host/api/generate" \
        -d "{\"model\":\"$model\",\"keep_alive\":-1}" >/dev/null; then
    echo "[Ollama] $model 올림 ($(( $(date +%s) - start )) s)"
else
    echo "[Ollama] $model 올리기 실패 ($(( $(date +%s) - start )) s) — 첫 질문이 늦거나 실패할 수 있습니다"
fi
exit 0
