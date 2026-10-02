#!/bin/bash
# Start Ubuntu's built-in remote login (RDP, port 3389).
# grdctl --system always calls pkexec, which fails in a terminal without a
# desktop session. Running grdctl as gnome-remote-desktop selects system mode
# directly. Connect with Remmina, protocol RDP.
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  exec sudo -- "$0" "$@"
fi

username="${1:-${SUDO_USER:-tb2}}"
client="${2:-192.168.1.4}"
home="$(getent passwd gnome-remote-desktop | cut -d: -f6)"
cert_dir="${home}/.local/share/gnome-remote-desktop"

if [[ -z "${RDP_PASSWORD:-}" ]]; then
  read -rsp "RDP password for ${username}: " RDP_PASSWORD </dev/tty
  echo
fi
if [[ -z "${RDP_PASSWORD}" ]]; then
  echo "Password is empty." >&2
  exit 1
fi

install -d -o gnome-remote-desktop -g gnome-remote-desktop -m 700 \
  "${home}/.local" "${home}/.local/share" "${cert_dir}"
if [[ ! -s "${cert_dir}/tls.key" || ! -s "${cert_dir}/tls.crt" ]]; then
  sudo -u gnome-remote-desktop -H openssl req -new -newkey rsa:2048 -days 720 \
    -nodes -x509 -subj "/CN=$(hostname)" \
    -keyout "${cert_dir}/tls.key" -out "${cert_dir}/tls.crt" >/dev/null 2>&1
fi
chown gnome-remote-desktop:gnome-remote-desktop "${cert_dir}/tls.key" "${cert_dir}/tls.crt"
chmod 600 "${cert_dir}/tls.key" "${cert_dir}/tls.crt"

install -d -m 755 /etc/systemd/system/gnome-remote-desktop.service.d
cat >/etc/systemd/system/gnome-remote-desktop.service.d/bind-port.conf <<'EOF'
[Service]
AmbientCapabilities=CAP_NET_BIND_SERVICE
EOF
systemctl daemon-reload

as_service=(sudo -u gnome-remote-desktop -H --)
"${as_service[@]}" grdctl rdp disable || true
"${as_service[@]}" grdctl rdp set-tls-key "${cert_dir}/tls.key"
"${as_service[@]}" grdctl rdp set-tls-cert "${cert_dir}/tls.crt"
"${as_service[@]}" grdctl rdp disable-view-only
"${as_service[@]}" grdctl rdp set-credentials "${username}" "${RDP_PASSWORD}"
"${as_service[@]}" grdctl rdp enable
unset RDP_PASSWORD

if [[ -n "${SUDO_USER:-}" && -S "/run/user/$(id -u "${SUDO_USER}")/bus" ]]; then
  runtime="/run/user/$(id -u "${SUDO_USER}")"
  sudo -u "${SUDO_USER}" env \
    XDG_RUNTIME_DIR="${runtime}" \
    DBUS_SESSION_BUS_ADDRESS="unix:path=${runtime}/bus" \
    systemctl --user disable --now gnome-remote-desktop-headless.service || true
fi

systemctl enable --now gdm.service
systemctl enable --now gnome-remote-desktop.service
systemctl restart gnome-remote-desktop.service

if command -v ufw >/dev/null && ufw status | grep -q "Status: active"; then
  ufw allow from "${client}" to any port 3389 proto tcp >/dev/null
fi

echo
"${as_service[@]}" grdctl status
echo
if ss -lnt | grep -q ':3389'; then
  echo "RDP is listening on port 3389."
else
  echo "Service started, but port 3389 is not open. See: journalctl -u gnome-remote-desktop -n 40" >&2
  exit 1
fi
echo "In Remmina choose RDP and connect to this computer, port 3389."
echo "Allowed client: ${client}. Login name: ${username}."
ip -4 -br addr show | awk '$1 != "lo" && $3 !~ /^192\.168\.88\./ {print "Address:", $3}'
