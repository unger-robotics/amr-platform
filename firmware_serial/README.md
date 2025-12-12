# AMR Platform - Autonomous Mobile Robot

> **Bachelor-Thesis:** Konzeption und Realisierung einer autonomen mobilen Roboterplattform
> **Status:** Phase 2 ✅ Abgeschlossen | **Firmware:** v0.5.0-pid

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
│  Container: serial_bridge                                   │
│  ├── ROS 2 Serial Bridge → /cmd_vel → ESP32                │
│  ├── /odom Publisher (nav_msgs/Odometry)                   │
│  └── TF Broadcast: odom → base_link                        │
└─────────────────────────────────────────────────────────────┘
                         │
                         │ USB-CDC Serial (115200 Baud)
                         │ Protokoll: V:<m/s>,W:<rad/s>\n
                         │            ODOM:<l>,<r>,<x>,<y>,<th>\n
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 XIAO (Echtzeit-Controller)                       │
│  ├── PID-Regler     → Geschwindigkeitsregelung pro Rad     │
│  ├── Dual-PWM       → Cytron MDD3A (D0-D3)                 │
│  ├── Encoder-ISR    → JGA25-370 Hall (D6, D7)              │
│  ├── Odometrie      → x, y, theta Berechnung               │
│  ├── IMU            → MPU6050 (I2C: D4, D5)                │
│  ├── LED-Status     → MOSFET IRLZ24N (D10)                 │
│  └── Failsafe       → 500ms Timeout                        │
└─────────────────────────────────────────────────────────────┘
```

---

## Aktueller Status

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| 0 | Fundament (Pi OS, Docker, Hailo) | ✅ Fertig |
| 1 | Motor-Test (Serial-Bridge → Motor) | ✅ Abgeschlossen |
| 2 | Odometrie + PID (Encoder, /odom) | ✅ **Abgeschlossen** |
| 3 | SLAM (LiDAR, Kartierung) | ◄── **Aktuell** |
| 4 | Navigation (Nav2, autonom) | ⬜ Offen |
| 5 | Kamera + AI (Hailo, YOLOv8) | ⬜ Offen |

---

## Phase 2 Highlights

### PID-Regelung (Closed-Loop)

| Metrik | Open-Loop | Mit PID | Verbesserung |
|--------|-----------|---------|--------------|
| Distanzfehler | 16% | **1.6%** | 10× besser |
| Drift | 14 cm | **0.5 cm** | 28× besser |
| Encoder-Sync | 2381/2470 | **1802/1802** | Perfekt |

### Kalibrierte Parameter

```cpp
// Encoder (10-Umdrehungen-Test)
#define TICKS_PER_REV_LEFT   374.3f
#define TICKS_PER_REV_RIGHT  373.6f

// PID-Regler (Bodentest)
#define PID_KP  13.0f
#define PID_KI  5.0f
#define PID_KD  0.01f
```

---

## Schnellstart

### 1. Repository klonen

```bash
git clone git@github.com:ju1-eu/amr-platform.git
cd amr-platform
```

### 2. ESP32 Firmware flashen (Mac/Linux)

```bash
cd firmware_serial
pio run --target upload
pio device monitor
```

**Erwartete Ausgabe:**

```
AMR-ESP32 v0.5.0-pid ready
PID: Kp=13.00 Ki=5.00 Kd=0.01
Commands: V:v,W:w | RESET_ODOM | PID:Kp,Ki,Kd | DEBUG:ON/OFF
```

### 3. Docker-Stack starten (Raspberry Pi)

```bash
ssh pi@rover
cd ~/amr-platform/docker
docker compose up -d

# Logs prüfen
docker compose logs -f serial_bridge
```

### 4. Teleop-Test

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Tasten:** `i`=Vorwärts, `,`=Rückwärts, `j`/`l`=Drehen, `k`=Stopp

### 5. Odometrie prüfen

```bash
# In separatem Terminal
ros2 topic echo /odom --once
```

---

## Verzeichnisstruktur

```
amr-platform/
├── firmware/                 # micro-ROS Firmware (nicht verwendet)
├── firmware_serial/          # Serial-Bridge Firmware ✅ v0.5.0-pid
│   ├── src/main.cpp          # PID-Regler, Odometrie
│   ├── include/config.h      # Kalibrierte Parameter
│   └── platformio.ini
├── firmware_test/            # Hardware-Validierung
├── ros2_ws/                  # ROS 2 Workspace
│   └── src/
│       ├── amr_description/  # URDF, Launch
│       ├── amr_bringup/      # Konfiguration
│       └── amr_serial_bridge/# Serial Bridge Node ✅ v0.4.0-odom
├── docker/                   # Container-Infrastruktur
│   ├── docker-compose.yml
│   └── perception/
├── docs/                     # Dokumentation
├── scripts/
│   └── tune_pid.sh           # PID-Tuning Script
├── config/                   # Runtime-Konfiguration
└── maps/                     # SLAM-Karten
```

---

## Serial-Protokoll (v0.5.0)

| Richtung | Format | Beispiel |
|----------|--------|----------|
| Host → ESP32 | `V:<m/s>,W:<rad/s>\n` | `V:0.20,W:0.00\n` |
| Host → ESP32 | `RESET_ODOM\n` | Odometrie zurücksetzen |
| Host → ESP32 | `PID:<Kp>,<Ki>,<Kd>\n` | `PID:13.0,5.0,0.01\n` |
| Host → ESP32 | `DEBUG:ON\n` | Velocity-Debug aktivieren |
| ESP32 → Host | `ODOM:<l>,<r>,<x>,<y>,<th>\n` | `ODOM:1802,1802,0.984,0.005,0.01` |
| ESP32 → Host | `VEL:<vl>,<vr>,<pwml>,<pwmr>\n` | Debug-Ausgabe (10 Hz) |
| ESP32 → Host | `OK:<v>,<w>` | Bestätigung |
| ESP32 → Host | `FAILSAFE:TIMEOUT` | Nach 500ms ohne Befehl |

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

## PID-Tuning

Das `tune_pid.sh` Script ermöglicht Live-Tuning auf dem Pi:

```bash
ssh pi@rover
cd ~/amr-platform/docker && docker compose down
~/tune_pid.sh
```

**Befehle:**

| Befehl | Funktion |
|--------|----------|
| `test` | 1m Geradeaus-Test |
| `p 13.0` | Kp setzen |
| `i 5.0` | Ki setzen |
| `d 0.01` | Kd setzen |
| `debug` | Velocity-Ausgabe anzeigen |
| `reset` | Odometrie zurücksetzen |

---

## Git-Workflow

```bash
# Mac: Entwickeln und pushen
git pull origin main
# ... arbeiten ...
git add . && git commit -m "feat: Beschreibung"
git push origin main

# Pi: Deployen
ssh pi@rover "cd ~/amr-platform && git pull && cd docker && docker compose up -d"
```

---

## Standards

- **REP-103:** SI-Einheiten (Meter, Radiant)
- **REP-105:** TF-Frames (map → odom → base_link)
- **Safety:** Failsafe-Timeout (500ms), LED-Feedback

---

## Lizenz

MIT License – siehe [LICENSE](LICENSE)

---

*Aktualisiert: 2025-12-12 | Autor: Jan Unger | <https://github.com/ju1-eu/amr-platform>*
