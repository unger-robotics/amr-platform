# Serial Bridge auf Raspberry Pi einrichten

> **Ziel:** ROS 2 `/cmd_vel` → Serial → ESP32 → Motoren
> **Voraussetzung:** ESP32 mit Serial-Bridge Firmware geflasht (v0.3.0-serial)
> **Status:** ✅ Validiert am 2025-12-12

---

## Schnellstart (Docker – empfohlen)

Wenn das Package bereits im Git-Repo liegt:

```bash
ssh pi@rover
cd ~/amr-platform
git pull origin main
cd docker
docker compose up -d
docker compose logs -f serial_bridge
```

**Erwartete Ausgabe:**

```
[INFO] [serial_bridge]: Serial Bridge gestartet: /dev/ttyACM0 @ 115200 baud
[INFO] [serial_bridge]: ESP32 bereit
```

---

## Schritt 1: Package auf Pi kopieren

### Option A: Via Git (empfohlen)

```bash
# Auf Mac: Committen
cd /Users/jan/daten/start/IoT/AMR/amr-platform
git add ros2_ws/src/amr_serial_bridge/
git commit -m "feat: ROS 2 Serial Bridge Package"
git push origin main

# Auf Pi: Pullen
ssh pi@rover
cd ~/amr-platform
git pull origin main
```

### Option B: Via SCP

```bash
# Von Mac aus
scp -r ros2_ws/src/amr_serial_bridge pi@rover:~/amr-platform/ros2_ws/src/
```

---

## Schritt 2: ESP32 verbinden

```bash
# USB-Verbindung prüfen
ls -la /dev/ttyACM*

# Erwartung: /dev/ttyACM0
```

> ⚠️ Falls `/dev/ttyACM0` nicht existiert: ESP32 neu anstecken oder USB-Kabel prüfen.

---

## Schritt 3: Docker-Stack starten

### docker-compose.yml (bereits konfiguriert)

Die Datei `~/amr-platform/docker/docker-compose.yml` enthält:

```yaml
services:
  # Perception Stack (ROS 2 Jazzy)
  perception:
    build: ./perception
    container_name: amr_perception
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev
      - /run/udev:/run/udev:ro
    environment:
      - ROS_DOMAIN_ID=0
    restart: unless-stopped

  # Serial Bridge (ersetzt micro-ROS Agent)
  serial_bridge:
    image: ros:jazzy-ros-base
    container_name: amr_serial_bridge
    network_mode: host
    privileged: true
    volumes:
      - /dev/ttyACM0:/dev/ttyACM0
      - ../ros2_ws/src/amr_serial_bridge:/ros2_ws/src/amr_serial_bridge:ro
      - /dev/shm:/dev/shm
    environment:
      - ROS_DOMAIN_ID=0
    working_dir: /ros2_ws
    command: >
      bash -c "
        apt-get update && apt-get install -y python3-pip python3-serial &&
        source /opt/ros/jazzy/setup.bash &&
        cd /ros2_ws &&
        colcon build --packages-select amr_serial_bridge &&
        source install/setup.bash &&
        ros2 launch amr_serial_bridge serial_bridge.launch.py
      "
    depends_on:
      - perception
    restart: unless-stopped
```

### Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose logs -f serial_bridge
```

---

## Schritt 4: Teleop testen

Teleop wird im **perception**-Container ausgeführt (dort ist ROS 2 vollständig installiert):

```bash
# Terminal: In Perception-Container
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:**

| Taste | Aktion |
|-------|--------|
| `i` | Vorwärts |
| `,` | Rückwärts |
| `j` | Links drehen |
| `l` | Rechts drehen |
| `k` | Stopp |
| `q` | Geschwindigkeit + |
| `z` | Geschwindigkeit - |

---

## Debugging

### Node läuft?

```bash
docker exec -it amr_perception bash
ros2 node list
# Erwartung: /serial_bridge
```

### Topic aktiv?

```bash
ros2 topic list
ros2 topic info /cmd_vel
# Erwartung: Subscription count: 1
```

### Manuell cmd_vel senden

```bash
# Vorwärts (0.2 m/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Drehen (0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once

# Stopp
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### Serial Monitor auf Pi (ohne Docker)

```bash
# Direkt auf dem Host
screen /dev/ttyACM0 115200

# Oder mit minicom
minicom -D /dev/ttyACM0 -b 115200
```

### Container-Logs

```bash
# Serial Bridge Logs
docker compose logs -f serial_bridge

# Perception Logs
docker compose logs -f perception
```

---

## Alternative: Ohne Docker

Falls Docker nicht verwendet werden soll:

```bash
ssh pi@rover

# pyserial installieren
pip3 install pyserial --break-system-packages

# Package bauen
cd ~/amr-platform/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select amr_serial_bridge
source install/setup.bash

# Bridge starten
ros2 launch amr_serial_bridge serial_bridge.launch.py
```

---

## Checkliste

- [x] ESP32 per USB am Pi angeschlossen
- [x] `/dev/ttyACM0` existiert
- [x] Docker Container laufen (`docker compose up -d`)
- [x] Serial Bridge zeigt "ESP32 bereit"
- [x] Teleop: Roboter fährt vorwärts/rückwärts
- [x] Teleop: Roboter dreht links/rechts
- [x] Failsafe: Container stoppen → Motoren stoppen nach 500ms

---

## Fehlerbehandlung

| Problem | Lösung |
|---------|--------|
| `/dev/ttyACM0` nicht vorhanden | ESP32 neu anstecken, `dmesg | tail` prüfen |
| "Permission denied" bei Serial | `sudo chmod 666 /dev/ttyACM0` oder User zu `dialout` Gruppe |
| Container startet nicht | `docker compose logs serial_bridge` prüfen |
| Motoren reagieren nicht | ESP32 Firmware prüfen, Serial Monitor testen |
| Failsafe ständig aktiv | Heartbeat-Problem, Bridge sendet nicht |

---

*Stand: 2025-12-12 | Version: 0.3.0 | Phase 1 abgeschlossen*
