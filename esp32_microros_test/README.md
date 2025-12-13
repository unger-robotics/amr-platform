# micro-ROS Test: ESP32-S3 ↔ Raspberry Pi 5

Validiert die bidirektionale Kommunikation zwischen ESP32 und Pi über micro-ROS.

## Architektur

```
┌─────────────────────────────────────────────────────────────────┐
│                      Raspberry Pi 5                             │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │              Docker: micro-ROS Agent                      │  │
│  │  ┌─────────────────┐    ┌─────────────────────────────┐   │  │
│  │  │   ROS 2 Jazzy   │    │     micro-ROS Agent         │   │  │
│  │  │   (Python 3.12) │◄──►│  Serial ↔ DDS Translation   │   │  │
│  │  └─────────────────┘    └──────────────┬──────────────┘   │  │
│  └────────────────────────────────────────┼──────────────────┘  │
│                                           │ /dev/ttyACM0        │
└───────────────────────────────────────────┼─────────────────────┘
                                            │ USB
┌───────────────────────────────────────────┼─────────────────────┐
│                      ESP32-S3              │                     │
│  ┌─────────────────────────────────────────▼─────────────────┐  │
│  │                  micro-ROS Client                         │  │
│  │  ┌─────────────────┐    ┌─────────────────────────────┐   │  │
│  │  │    Publisher    │    │        Subscriber           │   │  │
│  │  │ /esp32/heartbeat│    │       /esp32/led            │   │  │
│  │  │   (Int32: n++)  │    │   (Int32: 0=aus, 1=an)      │   │  │
│  │  └─────────────────┘    └─────────────────────────────┘   │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Voraussetzungen

### Raspberry Pi 5 (Raspbian/Debian Trixie)
- Docker installiert und lauffähig
- Benutzer in `docker` und `dialout` Gruppen

```bash
# Einmalig prüfen
groups $USER  # Sollte enthalten: docker dialout
```

### Entwicklungsrechner (macOS)
- PlatformIO installiert (VSCode Extension oder CLI)
- ESP32-S3 Toolchain

## Schritt-für-Schritt Anleitung

### 1. ESP32-S3 Firmware flashen

Auf dem Mac:

```bash
# Projekt klonen/kopieren
cd ~/Projects
# ... Dateien aus diesem Ordner kopieren

# Build und Upload
cd esp32_microros_test
pio run -t upload

# Serial Monitor (optional, für Debug)
pio device monitor
```

**Erwarteter Output auf Serial Monitor:**
```
========================================
micro-ROS Test-Firmware für ESP32-S3
========================================
LED Pin: GPIO 2
Publisher:  /esp32/heartbeat
Subscriber: /esp32/led
----------------------------------------
Warte auf micro-ROS Agent...
========================================
```

### 2. micro-ROS Agent auf Pi starten

Auf dem Pi:

```bash
# Test-Skript ausführen
./test_communication.sh /dev/ttyACM0

# Oder manuell:
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200
```

**Erwarteter Output:**
```
[INFO] Agent gefunden!
[INFO] Erstelle micro-ROS Entities...
[INFO] ✓ Verbunden mit Agent
[INFO] ✓ Publisher aktiv
[INFO] ✓ Subscriber aktiv
```

### 3. Kommunikation testen

In separatem Terminal auf dem Pi:

```bash
# Topics anzeigen
docker exec microros-agent ros2 topic list
# Erwartete Ausgabe:
#   /esp32/heartbeat
#   /esp32/led
#   /parameter_events
#   /rosout

# Heartbeat empfangen
docker exec -it microros-agent ros2 topic echo /esp32/heartbeat
# Erwartete Ausgabe (alle 500ms):
#   data: 42
#   ---
#   data: 43
#   ---

# LED einschalten
docker exec microros-agent ros2 topic pub --once /esp32/led std_msgs/msg/Int32 '{data: 1}'

# LED ausschalten
docker exec microros-agent ros2 topic pub --once /esp32/led std_msgs/msg/Int32 '{data: 0}'
```

## Troubleshooting

### Problem: "Warte auf micro-ROS Agent..." bleibt stehen

**Ursache:** Agent läuft nicht oder falscher Port.

**Lösung:**
```bash
# Auf Pi: Port prüfen
ls -la /dev/ttyACM*

# Agent-Logs prüfen
docker logs microros-agent
```

### Problem: Agent startet, aber keine Topics

**Ursache:** Baudrate-Mismatch oder Transport-Konfiguration.

**Lösung:**
1. ESP32 neu starten (Reset-Button)
2. Agent neu starten
3. Baudrate prüfen (beide Seiten 115200)

### Problem: "Entity-Erstellung fehlgeschlagen"

**Ursache:** micro-ROS Library nicht korrekt kompiliert.

**Lösung:**
```bash
# Clean Build
pio run -t clean
pio run -t upload
```

## Erweiterte Konfiguration

### Agent als Service (Autostart)

```bash
# Agent im Hintergrund mit Restart-Policy
docker run -d --restart=unless-stopped --name microros-agent \
    -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200
```

### Mehrere ESP32 gleichzeitig

Jeden ESP32 an unterschiedlichem Port, mehrere Agents:

```bash
docker run -d --name agent-motor -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200

docker run -d --name agent-sensor -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM1 -b 115200
```

## Nächste Schritte

Nach erfolgreicher Validierung:

1. **Motor-Control hinzufügen**: `/cmd_vel` Subscriber für Geschwindigkeit
2. **Encoder-Feedback**: `/odom` Publisher für Odometrie
3. **IMU-Integration**: `/imu` Publisher für Orientierung
4. **LIDAR-Bridge**: Separater ROS 2 Node für RPLIDAR

## Dateien

```
esp32_microros_test/
├── platformio.ini          # PlatformIO Konfiguration
├── extra_script.py         # Build-Hooks
├── src/
│   └── main.cpp            # ESP32 Firmware
├── test_communication.sh   # Pi Test-Skript
└── README.md               # Diese Datei
```
