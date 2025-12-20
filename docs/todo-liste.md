# ToDo-Liste AMR-Projekt

> **Stand:** 2025-12-20 | **Aktuelle Phase:** 3 (RPLidar)

---

## 📊 Phasen-Übersicht

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 1 | micro-ROS auf ESP32-S3 (USB-Serial) | ✅ Abgeschlossen |
| Phase 2 | Docker-Infrastruktur | ✅ Vorhanden |
| Phase 3 | RPLidar A1 Integration | ◄── **AKTUELL** |
| Phase 4 | EKF Sensor Fusion | ⬜ |
| Phase 5 | SLAM (slam_toolbox) | ⬜ |
| Phase 6 | Nav2 Autonome Navigation | ⬜ |

---

## ✅ Phase 1: micro-ROS ESP32-S3 – ABGESCHLOSSEN

### Firmware v3.2.0 (2025-12-20)

- [x] micro-ROS Client über USB-CDC (Serial)
- [x] Dual-Core FreeRTOS (Core 0: Control, Core 1: Comms)
- [x] `/cmd_vel` → Motorsteuerung (Cytron MDD3A)
- [x] `/odom_raw` → Odometrie (Pose2D)
- [x] `/esp32/heartbeat` → Lebenszeichen (1 Hz)
- [x] Failsafe (2000ms Timeout)
- [x] Feedforward-Steuerung (Gain=2.0)
- [x] PWM-Kanäle getauscht (A↔B)

### Konfiguration

| Parameter | Wert |
|-----------|------|
| Baudrate | 921600 |
| Feedforward Gain | 2.0 |
| PID | Deaktiviert (Kp=0) |
| Failsafe Timeout | 2000 ms |
| Loop Rate | 100 Hz |
| Odom Publish | 20 Hz |

### Testergebnisse

| Test | Status |
|------|--------|
| Vorwärts | ✅ |
| Rückwärts | ✅ |
| Drehen links | ✅ |
| Drehen rechts | ✅ |
| Failsafe | ✅ |
| Odom plausibel | ✅ |

---

## ✅ Phase 2: Docker-Infrastruktur – VORHANDEN

### Container

| Container | Image | Funktion | Status |
|-----------|-------|----------|--------|
| `amr_agent` | `microros/micro-ros-agent:humble` | Serial Agent | ✅ |
| `amr_dev` | Custom (ROS 2 Humble) | Workspace | ✅ |

### docker-compose.yml

```yaml
services:
  microros_agent:
    image: microros/micro-ros-agent:humble
    command: serial --dev /dev/ttyACM0 -b 921600

  amr_dev:
    build: .
    network_mode: host
```

---

## 🎯 Phase 3: RPLidar A1 Integration – AKTUELL

### Hardware

| Komponente | Port | Status |
|------------|------|--------|
| RPLidar A1 | `/dev/ttyUSB0` | ✅ Erkannt |

### Aufgaben

- [ ] `rplidar_ros` Package installieren
- [ ] Launch-File erstellen
- [ ] `/scan` Topic verifizieren
- [ ] Scan-Daten visualisieren (RViz2)
- [ ] Frame `laser` konfigurieren

### Geplante Topics

| Topic | Typ | Frequenz |
|-------|-----|----------|
| `/scan` | `sensor_msgs/LaserScan` | 5-10 Hz |

### Validierung

| Test | Kriterium | Status |
|------|-----------|--------|
| RPLidar startet | Motor dreht | ⬜ |
| `/scan` publiziert | Daten vorhanden | ⬜ |
| Range korrekt | 0.15m - 12m | ⬜ |
| RViz2 Visualisierung | Scan sichtbar | ⬜ |

---

## 📋 Phase 4: EKF Sensor Fusion

### Aufgaben

- [ ] `robot_localization` Package
- [ ] EKF Node konfigurieren
- [ ] Odom + IMU fusionieren (optional)
- [ ] `/odom` → `/odometry/filtered`
- [ ] TF: `odom` → `base_link`

### Geplante Topics

| Topic | Typ | Quelle |
|-------|-----|--------|
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF |
| `/tf` | `tf2_msgs/TFMessage` | EKF |

---

## 📋 Phase 5: SLAM (slam_toolbox)

### Aufgaben

- [ ] `slam_toolbox` konfigurieren
- [ ] Online Async SLAM
- [ ] Testraum kartieren
- [ ] Karte speichern (PGM + YAML)

### Abhängigkeiten

- Phase 3 (RPLidar) ✅
- Phase 4 (EKF) – optional, aber empfohlen

---

## 📋 Phase 6: Nav2 Autonome Navigation

### Aufgaben

- [ ] Nav2 Stack installieren
- [ ] AMCL Lokalisierung
- [ ] Costmaps konfigurieren
- [ ] Planner (NavFn / Smac)
- [ ] Controller (DWB / RPP)
- [ ] Punkt-zu-Punkt Navigation

### Abhängigkeiten

- Phase 5 (SLAM / Karte)

---

## 🔧 Hardware-Übersicht

| Komponente | Spezifikation | Status |
|------------|---------------|--------|
| Seeed XIAO ESP32-S3 | Dual-Core, USB-CDC | ✅ Aktiv |
| Cytron MDD3A | Dual-PWM, 4-16V | ✅ Aktiv |
| JGA25-370 (2×) | 12V DC + Encoder | ✅ Aktiv |
| Raspberry Pi 5 | 8GB, ROS 2 Humble | ✅ Aktiv |
| RPLidar A1 | 360° 2D Lidar | ✅ Erkannt |
| Hailo-8L | AI Accelerator | ⬜ Phase 6+ |

---

## 📁 Projekt-Struktur

```
amr-platform/
├── firmware/                 # ◄── AKTIV (v3.2.0)
│   ├── include/config.h
│   ├── src/main.cpp
│   └── platformio.ini
├── docker/
│   └── docker-compose.yml
├── ros2_ws/
│   └── src/
├── docs/
│   ├── phases/
│   │   └── 01-microros-esp32s3.md
│   └── phase1-befehle.md
└── scripts/
```

---

## 📅 Zeitplan

```
Woche:  1  2  3  4  5  6  7  8
        ════════════════════════════════
Phase 1 ████                             ✅ Abgeschlossen
Phase 2 ████                             ✅ Vorhanden
Phase 3       ████                       ◄── AKTUELL
Phase 4             ████
Phase 5                   ████
Phase 6                         ████
        ════════════════════════════════
```

---

## ✅ Checkliste: Phase 3 abgeschlossen wenn

- [ ] RPLidar startet automatisch
- [ ] `/scan` publiziert Daten
- [ ] RViz2 zeigt Scan korrekt
- [ ] Frame `laser` → `base_link` TF
- [ ] Dokumentation aktualisiert
- [ ] Code committet

---

## 🚀 Quick Start (Phase 1)

### Nach Pi Reboot

```bash
cd ~/amr-platform/docker
docker compose up -d
sleep 5
docker compose logs microros_agent --tail 5
```

### Motor-Test

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

---

*Aktualisiert: 2025-12-20 | Phase 1 abgeschlossen, Phase 3 bereit*
