#!/bin/bash
# 로컬 음성(⑬, vica-llm 유닛의 offline 쪽)이 뜨기 전에 모델을 Ollama 에 올려 둔다.
#
# 2026-11-17: 로컬로 띄운 8번 모두 음성 노드의 첫 호출(워밍업)이 15 s 에 시간 초과로 끊겼고,
# 그 순간 Ollama 가 적재를 그만뒀다(ollama_serve.log "client connection closed before
# llama-server finished loading, aborting load", 499). Ollama 를 새로 켜면 GPU 확인만 33~49 s 라
# 15 s 안에 올라올 수 없다. 여기서 시한을 넉넉히(기본 180 s) 두고 먼저 올리면, 음성 노드의
# 첫 호출은 이미 올라간 모델을 쓴다(올라간 뒤 1.5~3 s).
#
# 쓰는 법: vica_ollama_preload.sh [모델] [시한 초]
# 끝 코드는 늘 0 — 못 올려도 음성은 띄운다(첫 질문이 늦거나 실패할 수 있음을 알릴 뿐).
# 찍는 줄은 모드 노드가 읽는다(mode_logic.VoiceEventTracker: '[Ollama] … 올리기 실패').
model="${1:-midm2-mini}"
limit="${2:-180}"
host="${OLLAMA_HOST:-http://localhost:11434}"
start=$(date +%s)

# 서버가 포트를 열 때까지(최대 30 s). 실행 스크립트가 방금 nohup 으로 켰을 수 있다.
for _ in $(seq 1 30); do
    curl -sf --max-time 2 "$host/api/tags" >/dev/null && break
    sleep 1
done

# 떠 있는 Ollama 가 설정 없이 떴는지(다른 곳에서 먼저 켠 경우) — 모델이 내려가거나 목록을 놓칠 수 있다.
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
