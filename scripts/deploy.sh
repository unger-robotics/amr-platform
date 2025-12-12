#!/bin/bash
# ==============================================================================
# AMR Deploy Script - Synchronisiert GitHub → Pi und startet Container
# Speichern als: ~/amr-platform/scripts/deploy.sh
# ==============================================================================

set -e  # Bei Fehler abbrechen

# Farben für Output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}═══════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}       AMR Platform - Deploy Script                 ${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════${NC}"

# Projektverzeichnis
PROJECT_DIR="${HOME}/amr-platform"
cd "$PROJECT_DIR"

# 1. Git Status prüfen
echo -e "\n${GREEN}[1/4] Git Status prüfen...${NC}"
if [[ -n $(git status --porcelain) ]]; then
    echo -e "${YELLOW}⚠ Lokale Änderungen gefunden:${NC}"
    git status --short
    read -p "Änderungen committen? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        git add .
        read -p "Commit-Nachricht: " msg
        git commit -m "$msg"
        git push origin main
    fi
fi

# 2. Updates holen
echo -e "\n${GREEN}[2/4] Updates von GitHub holen...${NC}"
git pull origin main

# 3. Docker Container neu bauen (falls nötig)
echo -e "\n${GREEN}[3/4] Docker Container prüfen...${NC}"
cd "$PROJECT_DIR/docker"

if [[ "$1" == "--rebuild" ]]; then
    echo "Rebuild erzwungen..."
    docker compose build --no-cache
else
    docker compose build
fi

# 4. Container starten
echo -e "\n${GREEN}[4/4] Container starten...${NC}"
docker compose up -d

# Status anzeigen
echo -e "\n${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ Deployment abgeschlossen!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"

echo -e "\n${YELLOW}Container Status:${NC}"
docker compose ps

echo -e "\n${YELLOW}Logs anzeigen mit:${NC}"
echo "  docker compose logs -f"

echo -e "\n${YELLOW}micro-ROS Agent testen:${NC}"
echo "  ros2 node list"
