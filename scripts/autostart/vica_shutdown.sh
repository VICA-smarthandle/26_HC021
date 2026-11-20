#!/usr/bin/env bash
# 로봇 끄기 — 앱 '로봇 끄기'(사용자 2026-11-18). 모드 노드(/vica/mode/shutdown)가 일회성 유닛으로 띄운다:
#   systemd-run --user --unit=vica-shutdown --collect --no-block <bin>/vica_shutdown.sh
# 모드 노드도 기본 묶음이라 같이 내려간다 — 그래서 끄는 일은 이 따로 뜬 유닛이 한다.
#
#   주행 묶음 → 기본 묶음(유닛마다 정해 둔 정지 절차 그대로: SIGINT, motor 가 safety 보다 먼저)
#   → 디스크 기록(sync) → 시스템 종료.
# 묶음 정지가 실패해도 이어서 진행한다 — 시스템 종료가 남은 것을 모두 내린다.
# 시스템 종료 권한: polkit 규칙(install_poweroff_rule.sh, sudo 로 한 번). 없으면 스택만 내려가고
# sync 까지 한 뒤 멈춘다(그 상태에서 전원을 내려도 기록은 끝나 있다).
set -u
log() { echo "[로봇 끄기] $*"; }

# 마이크 점검 중이면 먼저 내린다(점검기가 내려가며 reSpeaker 칩 설정을 되돌린다). 없으면 그냥 지나간다.
systemctl --user stop vica-miccheck.service 2>/dev/null || true
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
