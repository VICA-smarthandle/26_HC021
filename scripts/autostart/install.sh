#!/bin/bash
set -eu

if [ "$(id -u)" -ne 0 ]; then
  echo "root 권한이 필요합니다: sudo bash $0" >&2
  exit 1
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_USER="${SUDO_USER:-ji_w}"
ROBOT_UID="$(id -u "$ROBOT_USER")"

echo "[1/4] can1 부팅 설정"
install -m 0755 "$HERE/vica_can_up.sh" /usr/local/sbin/vica_can_up.sh
install -m 0644 "$HERE/vica-can.service" /etc/systemd/system/vica-can.service
systemctl daemon-reload
systemctl enable vica-can.service
echo "      다음 부팅부터 적용됩니다. 지금 can1 상태는 그대로입니다."

echo "[2/4] linger ($ROBOT_USER)"
loginctl enable-linger "$ROBOT_USER"

echo "[3/4] 저널 보존"
mkdir -p /var/log/journal /etc/systemd/journald.conf.d
printf '[Journal]\nStorage=persistent\nSystemMaxUse=1G\n' \
  > /etc/systemd/journald.conf.d/vica.conf
systemd-tmpfiles --create --prefix /var/log/journal
systemctl restart systemd-journald

echo "[4/4] nice 한도 ($ROBOT_USER)"
printf '%s  -  nice  -10\n' "$ROBOT_USER" > /etc/security/limits.d/vica-nice.conf
rm -f "/etc/systemd/system/user@${ROBOT_UID}.service.d/vica-nice.conf"
rmdir "/etc/systemd/system/user@${ROBOT_UID}.service.d" 2>/dev/null || true
systemctl daemon-reload
echo "      다음 부팅부터 적용됩니다."

echo
echo "설치 끝. 확인:"
echo "  systemctl is-enabled vica-can.service"
echo "  loginctl show-user $ROBOT_USER -p Linger"
echo "  journalctl --disk-usage"
echo "  (재부팅 뒤) grep 'Max nice' /proc/\$(pgrep -x safety_supervis)/limits   → 30"
