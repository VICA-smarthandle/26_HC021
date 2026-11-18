#!/usr/bin/env bash
# 앱 '로봇 끄기'(사용자 2026-11-18)가 시스템을 끌 수 있게 로봇 사용자에게 종료 권한을 준다.
# 모드 노드는 로그인 세션 밖(linger) 사용자 유닛이라 polkit 규칙이 없으면 'systemctl poweroff' 가 거절된다.
# 이 장비의 polkit 은 0.105 라 .pkla 형식이다(JS rules.d 는 안 읽음). sudo 로 한 번:
#   sudo bash scripts/autostart/install_poweroff_rule.sh
# 되돌리기: sudo rm /etc/polkit-1/localauthority/50-local.d/50-vica-poweroff.pkla
set -eu
user="${SUDO_USER:-ji_w}"
dst=/etc/polkit-1/localauthority/50-local.d/50-vica-poweroff.pkla
mkdir -p "$(dirname "$dst")"
cat > "$dst" <<PKLA
[VICA 로봇 끄기 — $user 가 시스템 종료]
Identity=unix-user:$user
Action=org.freedesktop.login1.power-off;org.freedesktop.login1.power-off-multiple-sessions;org.freedesktop.login1.power-off-ignore-inhibit
ResultAny=yes
ResultInactive=yes
ResultActive=yes
PKLA
chmod 0644 "$dst"
echo "설치: $dst ($user)"
