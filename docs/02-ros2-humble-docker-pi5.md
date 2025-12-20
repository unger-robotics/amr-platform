---
title: "Phase 2 – ROS 2 Humble auf Pi 5 via Docker + micro-ROS Agent"
status: "completed"
updated: "2025-12-20"
version: "2.0"
depends_on:
  - "Phase 1 (micro-ROS auf ESP32-S3)"
next:
  - "Phase 3 (RPLidar A1)"
---

# Phase 2: ROS 2 Humble auf Raspberry Pi 5 (Docker) + micro-ROS Agent

## Zielbild & Definition of Done

### Zielbild

- Raspberry Pi 5 (Raspberry Pi OS 64-bit) betreibt **ROS 2 Humble** in **Docker**.
- micro-ROS Agent läuft reproduzierbar (Container), verbindet sich über **USB-Serial** zum ESP32-S3.
- Host-ROS kann:
  - `/cmd_vel` publizieren → Motor reagiert
  - `/odom_raw` empfangen → Werte plausibel
  - `/esp32/heartbeat` empfangen → Agent-Verbindung verifiziert

### DoD (verifiziert 2025-12-20)

- [x] `docker compose up` startet Container ohne manuelle Nacharbeit.
- [x] Agent verbindet sich stabil über `/dev/ttyACM0` mit 921600 Baud.
- [x] ROS Smoke-Tests sind grün:
  - [x] `ros2 topic list` zeigt `/cmd_vel`, `/odom_raw`, `/esp32/heartbeat`, `/esp32/led_cmd`
  - [x] `ros2 topic pub /cmd_vel ...` bewegt den Rover
  - [x] `ros2 topic echo /odom_raw` liefert kontinuierliche Werte
- [x] Failsafe stoppt Motoren nach 2s Timeout

---

## 1) Docker-Images (Regel)

| Container | Image | Funktion |
|-----------|-------|----------|
| `amr_agent` | `microros/micro-ros-agent:humble` | Serial Agent |
| `amr_dev` | Custom (ROS 2 Humble) | Workspace |

**Warum Humble statt Jazzy:**

- micro-ROS Agent für Humble stabiler auf arm64
- Kompatibilität mit bestehenden Packages (Nav2, slam_toolbox)

---

## 2) Host-Voraussetzungen

### 2.1 System

- Raspberry Pi 5, Raspberry Pi OS **64-bit** (Bookworm)
- Docker Engine + Docker Compose Plugin

### 2.2 USB-Serial prüfen

```bash
ls -l /dev/ttyACM*
# Erwartung: /dev/ttyACM0 (ESP32-S3)

ls -l /dev/ttyUSB*
# Erwartung: /dev/ttyUSB0 (RPLidar A1)
```

---

## 3) Repo-Struktur

```
amr-platform/
├── docker/
│   ├── docker-compose.yml
│   └── Dockerfile
├── ros2_ws/
│   └── src/
├── firmware/
│   ├── src/main.cpp
│   └── include/config.h
└── docs/
    └── phases/
```

---

## 4) Docker Compose

### docker-compose.yml

```yaml
services:
  microros_agent:
    image: microros/micro-ros-agent:humble
    container_name: amr_agent
    network_mode: host
    privileged: true
    restart: always
    command: serial --dev /dev/ttyACM0 -b 921600
    devices:
      - /dev/ttyACM0:/dev/ttyACM0

  amr_dev:
    build: .
    container_name: amr_base
    network_mode: host
    privileged: true
    volumes:
      - ../ros2_ws:/root/ros2_ws
    command: tail -f /dev/null
```

### Dockerfile

```dockerfile
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-tf2-tools \
    ros-humble-xacro \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws
```

---

## 5) Befehle

### 5.1 Nach Pi Reboot

```bash
cd ~/amr-platform/docker
docker compose up -d
sleep 5
docker compose logs microros_agent --tail 5
```

**Erwartung:** `running... | fd: 3`

### 5.2 Nach ESP32 Reboot

```bash
docker compose restart microros_agent
sleep 5
docker compose logs microros_agent --tail 5
```

### 5.3 In Container gehen

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
```

---

## 6) Smoke-Tests

### 6.1 Topics prüfen

```bash
ros2 topic list
```

**Erwartung:**

```
/cmd_vel
/esp32/heartbeat
/esp32/led_cmd
/odom_raw
/parameter_events
/rosout
```

### 6.2 Heartbeat

```bash
ros2 topic echo /esp32/heartbeat
```

**Erwartung:** Counter steigt ~1 Hz

### 6.3 Odometrie

```bash
ros2 topic echo /odom_raw --once
```

**Erwartung:** x, y, theta Werte

### 6.4 Motor-Test (⚠️ Räder aufbocken!)

```bash
# Vorwärts
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10

# Ctrl+C → Failsafe stoppt nach 2s
```

---

## 7) Troubleshooting

| Problem | Ursache | Lösung |
|---------|---------|--------|
| Agent sieht ESP32 nicht | Device-Pfad falsch | `ls /dev/ttyACM*` prüfen |
| Topics fehlen | Agent nicht verbunden | `docker compose restart microros_agent` |
| ROS sieht keine Topics | Domain-ID Mismatch | `network_mode: host` nutzen |
| Motor reagiert nicht | Failsafe greift | Timeout prüfen (2000ms) |

---

## 8) Verifizierte Konfiguration

| Parameter | Wert |
|-----------|------|
| ROS-Version | Humble |
| Agent-Image | `microros/micro-ros-agent:humble` |
| Baudrate | 921600 |
| Device | `/dev/ttyACM0` |
| Container | `amr_agent`, `amr_dev` |
| Network | host |

---

## 9) Changelog

| Version | Datum | Änderungen |
|---------|-------|------------|
| v2.0 | 2025-12-20 | Humble statt Jazzy, 921600 Baud, Container-Namen aktualisiert, Status: abgeschlossen |
| v1.0 | 2025-12-19 | Initiale Jazzy-Version (überholt) |
