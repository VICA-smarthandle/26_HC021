#!/usr/bin/env bash
set -u
log() { echo "[로봇 끄기] $*"; }

systemctl --user stop vica-miccheck.service 2>/dev/null || true
log "마이크 점검 내림"
log "주행 묶음 내림"
systemctl --user stop vica-drive.target || log "주행 묶음 정지 실패 — 이어서 진행"
log "기본 묶음 내림"
systemctl --user stop vica-base.target || log "기본 묶음 정지 실패 — 이어서 진행"
log "디스크 기록(sync)"
sync
log "시스템 종료"
if ! systemctl poweroff; then
  log "시스템 종료 권한이 없습니다 — scripts/autostart/install_poweroff_rule.sh 를 sudo 로 한 번 실행하세요."
  log "스택은 내려갔고 디스크 기록은 끝났습니다. 전원을 내려도 됩니다."
  exit 1
fi
