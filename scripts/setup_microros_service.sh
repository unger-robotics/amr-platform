#!/bin/bash
# =============================================================================
# micro-ROS Agent Service Setup für Raspberry Pi 5
# Installiert und aktiviert den systemd Service
# =============================================================================

set -e

SERVICE_FILE="/etc/systemd/system/microros-agent.service"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== micro-ROS Agent Service Setup ==="
echo ""

# Prüfen ob als root
if [ "$EUID" -ne 0 ]; then
    echo "Bitte mit sudo ausführen:"
    echo "  sudo $0"
    exit 1
fi

# Prüfen ob Docker läuft
if ! docker info &>/dev/null; then
    echo "❌ Docker läuft nicht!"
    exit 1
fi

# Prüfen ob Image vorhanden
if ! docker image inspect microros/micro-ros-agent:humble &>/dev/null; then
    echo "📦 micro-ROS Agent Image wird geladen..."
    docker pull microros/micro-ros-agent:humble
fi

# Service-Datei kopieren
echo "📝 Service-Datei wird installiert..."
cat > "$SERVICE_FILE" << 'EOF'
[Unit]
Description=micro-ROS Agent for ESP32
Documentation=https://micro.ros.org/
After=docker.service
Requires=docker.service

[Service]
Type=simple
Restart=always
RestartSec=5

# Container vorher aufräumen
ExecStartPre=-/usr/bin/docker stop microros-agent
ExecStartPre=-/usr/bin/docker rm microros-agent

# Agent starten
ExecStart=/usr/bin/docker run --rm --name microros-agent \
    -v /dev:/dev \
    --privileged \
    --net=host \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyACM0 -b 115200

# Sauberes Beenden
ExecStop=/usr/bin/docker stop microros-agent

[Install]
WantedBy=multi-user.target
EOF

# systemd neu laden
echo "🔄 systemd wird neu geladen..."
systemctl daemon-reload

# Service aktivieren
echo "✅ Service wird aktiviert..."
systemctl enable microros-agent

# Service starten
echo "🚀 Service wird gestartet..."
systemctl start microros-agent

# Status anzeigen
echo ""
echo "=== Service Status ==="
systemctl status microros-agent --no-pager || true

echo ""
echo "=== Nützliche Befehle ==="
echo "  Status:    sudo systemctl status microros-agent"
echo "  Logs:      sudo journalctl -u microros-agent -f"
echo "  Stoppen:   sudo systemctl stop microros-agent"
echo "  Starten:   sudo systemctl start microros-agent"
echo "  Deaktiv.:  sudo systemctl disable microros-agent"
echo ""
echo "✅ Setup abgeschlossen!"
