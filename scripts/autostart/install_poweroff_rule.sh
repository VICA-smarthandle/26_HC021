#!/usr/bin/env bash
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
