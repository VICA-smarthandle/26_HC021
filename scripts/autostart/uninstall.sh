#!/bin/bash
# VICA 자동 기동 0단계 되돌리기.
#
#   sudo bash scripts/autostart/uninstall.sh
#
# can1 부팅 설정과 linger 를 끈다. 지금 떠 있는 can1 링크는 건드리지 않는다.
# 저널 보존(/var/log/journal)은 로그가 들어 있으므로 지우지 않는다.
set -eu

if [ "$(id -u)" -ne 0 ]; then
  echo "root 권한이 필요합니다: sudo bash $0" >&2
  exit 1
fi

ROBOT_USER="${SUDO_USER:-ji_w}"

systemctl disable vica-can.service 2>/dev/null || true
rm -f /etc/systemd/system/vica-can.service /usr/local/sbin/vica_can_up.sh
systemctl daemon-reload
loginctl disable-linger "$ROBOT_USER"
rm -f /etc/security/limits.d/vica-nice.conf
rm -f "/etc/systemd/system/user@$(id -u "$ROBOT_USER").service.d/vica-nice.conf"
systemctl daemon-reload

echo "되돌림 끝. 사용자 유닛은 따로 끕니다:"
echo "  systemctl --user disable --now vica-base.target"
