#!/bin/bash
# install_udev_rules.sh - Installiert udev-Regeln fuer AMR-Geraete
#
# Ausfuehren auf dem Pi:
#   chmod +x install_udev_rules.sh
#   sudo ./install_udev_rules.sh

set -e

RULES_FILE="99-amr-usb.rules"
DEST="/etc/udev/rules.d/${RULES_FILE}"

echo "=== AMR udev-Regeln Installation ==="

# Pruefen ob als root ausgefuehrt
if [ "$EUID" -ne 0 ]; then
    echo "Fehler: Bitte mit sudo ausfuehren"
    exit 1
fi

# Regel-Datei kopieren
cp "${RULES_FILE}" "${DEST}"
echo "✓ Regel kopiert nach ${DEST}"

# udev neu laden
udevadm control --reload-rules
udevadm trigger
echo "✓ udev-Regeln neu geladen"

# Symlinks pruefen
echo ""
echo "=== Symlinks pruefen ==="
echo "RPLidar:"
ls -la /dev/rplidar 2>/dev/null || echo "  (nicht angeschlossen)"
echo "ESP32:"
ls -la /dev/esp32 2>/dev/null || echo "  (nicht angeschlossen)"

echo ""
echo "=== Fertig ==="
echo "Nach dem naechsten Einstecken der Geraete:"
echo "  - RPLidar: /dev/rplidar"
echo "  - ESP32:   /dev/esp32"
