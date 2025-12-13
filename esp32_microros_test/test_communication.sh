#!/bin/bash
# =============================================================================
# micro-ROS Kommunikationstest auf Raspberry Pi 5
# Startet Agent und testet bidirektionale Kommunikation mit ESP32
# =============================================================================

set -e

# Konfiguration
SERIAL_PORT="${1:-/dev/ttyACM0}"
BAUDRATE="${2:-115200}"
IMAGE="microros/micro-ros-agent:jazzy"

# Farben
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo ""
echo "=============================================="
echo -e "${CYAN}micro-ROS Kommunikationstest${NC}"
echo "=============================================="
echo "  Port:     $SERIAL_PORT"
echo "  Baudrate: $BAUDRATE"
echo "=============================================="
echo ""

# Prüfe ob Port existiert
if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${YELLOW}[WARN] Port $SERIAL_PORT nicht gefunden!${NC}"
    echo "Verfügbare Ports:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  Keine gefunden"
    exit 1
fi

# Funktion: Agent im Hintergrund starten
start_agent() {
    echo -e "${YELLOW}[1/4] Starte micro-ROS Agent...${NC}"
    
    # Eventuell laufenden Agent stoppen
    docker stop microros-agent 2>/dev/null || true
    docker rm microros-agent 2>/dev/null || true
    
    # Agent starten
    docker run -d --name microros-agent \
        -v /dev:/dev --privileged --net=host \
        $IMAGE serial --dev $SERIAL_PORT -b $BAUDRATE
    
    echo -e "${GREEN}  ✓ Agent gestartet${NC}"
    sleep 2
}

# Funktion: ROS 2 Container für Tests
run_ros2_cmd() {
    docker exec -it microros-agent ros2 $@
}

# Agent starten
start_agent

# Topics auflisten
echo ""
echo -e "${YELLOW}[2/4] Warte auf ESP32 Topics (10s)...${NC}"
sleep 5

echo ""
echo -e "${YELLOW}[3/4] Aktive Topics:${NC}"
docker exec microros-agent ros2 topic list 2>/dev/null || echo "  (Noch keine Topics - ESP32 verbunden?)"

echo ""
echo -e "${YELLOW}[4/4] Node-Info:${NC}"
docker exec microros-agent ros2 node list 2>/dev/null || echo "  (Noch keine Nodes)"

echo ""
echo "=============================================="
echo -e "${GREEN}Agent läuft!${NC}"
echo "=============================================="
echo ""
echo "Nützliche Befehle (in separatem Terminal):"
echo ""
echo "# Heartbeat vom ESP32 anzeigen:"
echo "docker exec -it microros-agent ros2 topic echo /esp32/heartbeat"
echo ""
echo "# LED einschalten:"
echo "docker exec microros-agent ros2 topic pub --once /esp32/led std_msgs/msg/Int32 '{data: 1}'"
echo ""
echo "# LED ausschalten:"
echo "docker exec microros-agent ros2 topic pub --once /esp32/led std_msgs/msg/Int32 '{data: 0}'"
echo ""
echo "# Agent-Logs anzeigen:"
echo "docker logs -f microros-agent"
echo ""
echo "# Agent stoppen:"
echo "docker stop microros-agent && docker rm microros-agent"
echo ""
