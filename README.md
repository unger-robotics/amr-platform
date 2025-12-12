# AMR Platform - Autonomous Mobile Robot

> **Bachelor-Thesis:** Konzeption und Realisierung einer autonomen mobilen Roboterplattform
> **Status:** Phase 1 (Motor-Test) | **Firmware:** v1.1.0

Autonome mobile Roboterplattform mit ROS 2 Jazzy auf Raspberry Pi 5 und ESP32-S3 Echtzeit-Controller.

---

## Architektur

```
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Raspberry Pi OS Lite + Docker)            │
│  ├── libcamera 0.3+ → IMX296 Global Shutter Kamera         │
│  ├── HailoRT 4.23   → Hailo-8L AI Beschleuniger (13 TOPS)  │
│  └── Docker         → ROS 2 Jazzy Container                │
├─────────────────────────────────────────────────────────────┤
│  Container: perception                                      │
│  ├── Nav2, SLAM Toolbox, robot_localization                │
│  ├── rplidar_ros, camera_ros                               │
│  └── amr_description (URDF), amr_bringup (Launch)          │
├─────────────────────────────────────────────────────────────┤
│  Container: micro_ros                                       │
│  └── micro-ROS Agent → USB-Serial Bridge                   │
└─────────────────────────────────────────────────────────────┘
                         │
                         │ USB-CDC Serial (115200 Baud)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 XIAO (Echtzeit-Controller, FreeRTOS)             │
│  ├── Dual-PWM       → Cytron MDD3A (D0-D3)                 │
│  ├── Encoder        → JGA25-370 Hall (D6, D7)              │
│  ├── IMU            → MPU6050 (I2C: D4, D5)                │
│  ├── LED-Status     → MOSFET IRLZ24N (D10)                 │
│  └── micro-ROS      → /cmd_vel Subscriber                  │
└─────────────────────────────────────────────────────────────┘
```

---

## Aktueller Status

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| 0 | Fundament (Pi OS, Docker, Hailo) | ✅ Fertig |
| 1 | Motor-Test (micro-ROS → Motor) | 🔄 **Aktuell** |
| 2 | Odometrie (Encoder, /odom) | ⬜ Offen |
| 3 | SLAM (LiDAR, Kartierung) | ⬜ Offen |
| 4 | Navigation (Nav2, autonom) | ⬜ Offen |
| 5 | Kamera + AI (Hailo, YOLOv8) | ⬜ Offen |

---

## Schnellstart

### 1. Repository klonen

```bash
git clone git@github.com:ju1-eu/amr-platform.git
cd amr-platform
```

### 2. ESP32 Firmware flashen (Mac/Linux)

```bash
cd firmware
pio run --target upload
```

### 3. Docker-Stack starten (Raspberry Pi)

```bash
cd ~/amr-platform/docker
docker compose up -d

# Logs prüfen
docker compose logs -f micro_ros
```

### 4. Motor-Test

```bash
# Node prüfen
ros2 node list
# Erwartung: /amr_esp32

# Motor vorwärts
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}" --once

# Drehung
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}" --once
```

---

## Verzeichnisstruktur

```
amr-platform/
├── firmware/                 # ESP32-S3 XIAO (PlatformIO)
│   ├── src/main.cpp          # FreeRTOS Tasks, micro-ROS
│   ├── include/config.h      # Hardware-Parameter
│   └── platformio.ini
├── ros2_ws/                  # ROS 2 Workspace
│   └── src/
│       ├── amr_description/  # URDF, Launch
│       └── amr_bringup/      # Konfiguration
├── docker/                   # Container-Infrastruktur
│   ├── docker-compose.yml
│   └── Dockerfile
├── docs/                     # Dokumentation
├── config/                   # Runtime-Konfiguration
├── maps/                     # SLAM-Karten
└── scripts/                  # Deploy-Scripts
```

---

## Dokumentation

| Nr | Dokument | Inhalt |
|----|----------|--------|
| 01 | `01-Pi-OS-flashen.md` | Raspberry Pi OS, SSH, Docker |
| 02 | `02-hailo-setup.md` | Hailo-8L Treiber, Benchmark |
| 03 | `03-ros2-docker.md` | ROS 2 Container, URDF |
| 04 | `04-esp32-firmware.md` | PlatformIO, micro-ROS, **v1.1.0** |
| 05 | `05-git-vscode-platformio.md` | Entwicklungsumgebung |
| 06 | `06-git-workflow.md` | Git Mac ↔ GitHub ↔ Pi |

---

## Hardware

| Komponente | Modell | Funktion | Preis |
|------------|--------|----------|-------|
| Compute | Raspberry Pi 5 (8GB) | ROS 2 Host, SLAM | 82,90 € |
| AI | Hailo-8L Kit | Objekterkennung (13 TOPS) | 78,85 € |
| LiDAR | RPLIDAR A1 | 360° Scan, 8k Samples/s | 89,90 € |
| Kamera | IMX296 Global Shutter | Bewegungserkennung | 58,90 € |
| MCU | ESP32-S3 XIAO | Echtzeit-Control | 8,50 € |
| Motoren | JGA25-370 (2×) | 170 RPM, Encoder | 20,37 € |
| Treiber | Cytron MDD3A | Dual-PWM, 3A | 8,50 € |

**Gesamtkosten:** 482,48 € (35% unter Referenz-Budget)

---

## Firmware v1.1.0 Highlights

| Feature | Beschreibung |
|---------|--------------|
| **Dual-PWM** | Korrekte MDD3A-Ansteuerung (nicht PWM+DIR) |
| **FreeRTOS** | Dual-Core Tasks (LED auf Core 0, ROS auf Core 1) |
| **Mutex** | Thread-sichere Variablen |
| **Failsafe** | Motoren stoppen nach 500 ms ohne /cmd_vel |
| **Watchdog** | ESP32 Reset nach 5 s Blockierung |
| **LED-Status** | Breathing, Blinken, SOS-Pattern |

---

## Pin-Belegung ESP32-S3 XIAO

| Komponente | Signal | Pin | PWM-Kanal |
|------------|--------|-----|-----------|
| Motor Links | PWM A (vorwärts) | D0 | CH 0 |
| Motor Links | PWM B (rückwärts) | D1 | CH 1 |
| Motor Rechts | PWM A (vorwärts) | D2 | CH 2 |
| Motor Rechts | PWM B (rückwärts) | D3 | CH 3 |
| IMU | SDA | D4 | – |
| IMU | SCL | D5 | – |
| Encoder Links | Phase A | D6 | – |
| Encoder Rechts | Phase A | D7 | – |
| LED-Strip | MOSFET Gate | D10 | CH 4 |

---

## Git-Workflow

```bash
# Mac: Entwickeln und pushen
git pull origin main
# ... arbeiten ...
git add . && git commit -m "feat: Beschreibung"
git push origin main

# Pi: Deployen
cd ~/amr-platform
git pull origin main
docker compose up -d
```

---

## Standards

- **REP-103:** SI-Einheiten (Meter, Radiant)
- **REP-105:** TF-Frames (map → odom → base_link)
- **Safety:** Failsafe-Timeout, Watchdog

---

## Lizenz

MIT License – siehe [LICENSE](LICENSE)

---

*Erstellt: 2025-12-12 | Autor: Jan Unger*
