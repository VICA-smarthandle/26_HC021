#!/bin/bash
# VICA 자동 기동 0단계 — root 권한이 필요한 것만 한 번 설치한다.
#
#   sudo bash scripts/autostart/install.sh
#
#   1. can1 부팅 설정   vica-can.service (다음 부팅부터. 지금 링크 상태는 바꾸지 않는다)
#   2. linger           로그인하지 않아도 부팅 직후 사용자 유닛이 뜬다
#   3. 저널 보존        /var/log/journal (지금은 재부팅하면 로그가 사라진다), 상한 1G
#   4. nice 한도        사용자 유닛이 nice -10 까지 올릴 수 있게(safety·motor 만 쓴다). 재부팅 뒤 적용
#
# 되돌리기: sudo bash scripts/autostart/uninstall.sh
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

echo "[4/4] nice 한도 (user@${ROBOT_UID})"
# safety·motor 실행 스크립트가 renice -10 으로 다른 노드보다 CPU 를 먼저 받는다(2026-11-16).
# 주행 묶음 기동 40 s 동안 8코어가 100 % 라 safety 출력이 p99 154 ms 로 늘었다.
# 사용자 관리자가 다시 뜰 때(재부팅) 적용된다. 지금 관리자를 restart 하면 사용자 유닛이 전부 내려간다.
mkdir -p "/etc/systemd/system/user@${ROBOT_UID}.service.d"
printf '[Service]\nLimitNICE=-10\n' > "/etc/systemd/system/user@${ROBOT_UID}.service.d/vica-nice.conf"
systemctl daemon-reload
echo "      다음 부팅부터 적용됩니다."

echo
echo "설치 끝. 확인:"
echo "  systemctl is-enabled vica-can.service"
echo "  loginctl show-user $ROBOT_USER -p Linger"
echo "  journalctl --disk-usage"
echo "  (재부팅 뒤) grep 'Max nice' /proc/\$(pgrep -x safety_supervis)/limits   → 30"
