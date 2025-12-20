# AMR Implementierungsplan

## Vom Schaltplan zur autonomen Navigation

> **Version:** 2.0 | **Stand:** 2025-12-20 | **Firmware:** v3.2.0

---

## Das Grundprinzip: Vertikale Scheiben statt horizontaler Schichten

Ein häufiger Fehler bei Robotik-Projekten: Man baut zuerst die gesamte Hardware auf, dann die gesamte Firmware, dann die gesamten Treiber – und am Ende, beim ersten Integrationstest, funktioniert nichts. Die Fehlersuche wird zum Albtraum, weil alles gleichzeitig neu ist.

**Unser Ansatz:** Wir schneiden das System in *vertikale Scheiben*. Jede Phase liefert ein lauffähiges Teilsystem, das wir testen können, bevor die nächste Komplexitätsstufe hinzukommt.

```
Klassisch (riskant):          Unser Weg (inkrementell):

┌──────────────────┐          Phase 1: ──────────────────► ✅
│    Navigation    │                   micro-ROS + Motor
├──────────────────┤
│   Wahrnehmung    │          Phase 2: ──────────────────► ✅
├──────────────────┤                   Docker-Infrastruktur
│    Firmware      │
├──────────────────┤          Phase 3: ──────────────────► ◄── AKTUELL
│    Hardware      │                   RPLidar + Scan
└──────────────────┘
       ↓                      Phase 4: ──────────────────►
  Big Bang Test                        EKF + Sensor Fusion
  (Chaos)
                              Phase 5: ──────────────────►
                                       SLAM + Karte

                              Phase 6: ──────────────────►
                                       Nav2 + Autonomie
```

---

## Phasen-Übersicht

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 1 | micro-ROS auf ESP32-S3 (USB-Serial) | ✅ Abgeschlossen |
| Phase 2 | Docker-Infrastruktur | ✅ Vorhanden |
| Phase 3 | RPLidar A1 Integration | ◄── **AKTUELL** |
| Phase 4 | EKF Sensor Fusion | ⬜ |
| Phase 5 | SLAM (slam_toolbox) | ⬜ |
| Phase 6 | Nav2 Autonome Navigation | ⬜ |

---

## Phase 1: micro-ROS auf ESP32-S3 ✅

**Ziel:** Native ROS 2 Kommunikation über USB-Serial mit Dual-Core FreeRTOS Architektur.

**Status:** ✅ Abgeschlossen (2025-12-20) | Firmware v3.2.0

### 1.1 Architektur

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 (micro-ROS Client)                                │
│                                                             │
│  Core 0: Control Task (100 Hz)                              │
│    - Feedforward-Steuerung (Gain=2.0)                       │
│    - Encoder-Auswertung (ISR)                               │
│    - Odometrie-Integration                                  │
│    - Failsafe-Check (2000ms Timeout)                        │
│                                                             │
│  Core 1: Communication (micro-ROS)                          │
│    - Executor Spin                                          │
│    - Odom Publish @ 20 Hz                                   │
│    - Heartbeat Publish @ 1 Hz                               │
└─────────────────────────────────────────────────────────────┘
                            │
                      USB-CDC (921600 Baud)
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Docker)                                    │
│  Container: amr_agent (micro-ros-agent)                     │
│  Container: amr_dev (ROS 2 Humble)                          │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Topics

| Topic | Typ | Richtung | Beschreibung |
|-------|-----|----------|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Geschwindigkeitsbefehle |
| `/odom_raw` | `geometry_msgs/Pose2D` | Pub | Odometrie (x, y, theta) |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | Lebenszeichen |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | LED-Steuerung |

### 1.3 Konfiguration

| Parameter | Wert |
|-----------|------|
| Baudrate | 921600 |
| Feedforward Gain | 2.0 |
| PID | Deaktiviert (Kp=0) |
| Failsafe Timeout | 2000 ms |
| PWM-Kanäle | Getauscht (A↔B) |

### 1.4 Testergebnisse

| Test | Status |
|------|--------|
| Vorwärts | ✅ |
| Rückwärts | ✅ |
| Drehen links | ✅ |
| Drehen rechts | ✅ |
| Failsafe (2s) | ✅ |
| Odom plausibel | ✅ |

### 1.5 Cytron MDD3A – Dual-PWM Steuerung

> ⚠️ **Kritisch:** Der MDD3A verwendet **kein** DIR-Pin, sondern zwei PWM-Signale pro Motor!

| M1A (PWM) | M1B (PWM) | Ergebnis |
|-----------|-----------|----------|
| 200 | 0 | Vorwärts |
| 0 | 200 | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |

**Meilenstein Phase 1:** ✅ micro-ROS funktioniert, alle Richtungen getestet, Failsafe aktiv.

---

## Phase 2: Docker-Infrastruktur ✅

**Ziel:** Container-basierte ROS 2 Umgebung für einfaches Deployment.

**Status:** ✅ Vorhanden

### 2.1 Container

| Container | Image | Funktion |
|-----------|-------|----------|
| `amr_agent` | `microros/micro-ros-agent:humble` | Serial Agent |
| `amr_dev` | Custom (ROS 2 Humble) | Workspace |

### 2.2 docker-compose.yml

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

### 2.3 Quick Start

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

**Meilenstein Phase 2:** ✅ Docker-Container starten automatisch, Agent verbindet.

---

## Phase 3: RPLidar A1 Integration ◄── AKTUELL

**Ziel:** 360° Laserscan für Umgebungswahrnehmung.

**Status:** 🔜 Bereit (`/dev/ttyUSB0` erkannt)

### 3.1 Hardware

| Komponente | Port | Status |
|------------|------|--------|
| RPLidar A1 | `/dev/ttyUSB0` | ✅ Erkannt |

### 3.2 Aufgaben

- [ ] `rplidar_ros` Package installieren
- [ ] Launch-File erstellen
- [ ] `/scan` Topic verifizieren
- [ ] Frame `laser` → `base_link` TF
- [ ] RViz2 Visualisierung

### 3.3 Geplante Topics

| Topic | Typ | Frequenz |
|-------|-----|----------|
| `/scan` | `sensor_msgs/LaserScan` | 5-10 Hz |

### 3.4 Launch-File (geplant)

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

**Meilenstein Phase 3:** `/scan` publiziert, Daten in RViz2 sichtbar.

---

## Phase 4: EKF Sensor Fusion

**Ziel:** Robuste Odometrie durch Fusion von Encoder-Daten (später + IMU).

### 4.1 Aufgaben

- [ ] `robot_localization` Package
- [ ] EKF Node konfigurieren
- [ ] `/odom_raw` → `/odometry/filtered`
- [ ] TF: `odom` → `base_link`
- [ ] Optional: IMU Integration (MPU6050)

### 4.2 Geplante Topics

| Topic | Typ | Quelle |
|-------|-----|--------|
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF |
| `/tf` | `tf2_msgs/TFMessage` | EKF |

**Meilenstein Phase 4:** TF-Baum korrekt, gefilterte Odometrie stabil.

---

## Phase 5: SLAM (slam_toolbox)

**Ziel:** Der Roboter baut eine Karte seiner Umgebung.

### 5.1 Aufgaben

- [ ] `slam_toolbox` konfigurieren
- [ ] Online Async SLAM
- [ ] Testraum kartieren
- [ ] Karte speichern (PGM + YAML)

### 5.2 Launch

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=slam_params.yaml
```

**Meilenstein Phase 5:** Eine speicherbare Karte des Testraums existiert.

---

## Phase 6: Nav2 Autonome Navigation

**Ziel:** Wir setzen ein Ziel auf der Karte, der Roboter fährt autonom hin.

### 6.1 Nav2 Stack

| Komponente | Funktion |
|------------|----------|
| **AMCL** | Lokalisierung auf bekannter Karte |
| **Planner Server** | Globaler Pfad (A* / Dijkstra) |
| **Controller Server** | Lokale Hindernisvermeidung |
| **Costmap** | Hinderniskarte aus Sensordaten |
| **BT Navigator** | Verhaltenssteuerung |

**Meilenstein Phase 6:** Roboter navigiert autonom, weicht Hindernissen aus.

---

## Zukünftige Erweiterungen

### Kamera & AI (Optional)

- IMX296 Global Shutter Kamera
- YOLOv8 auf Hailo-8L
- Personen-Erkennung → Stopp-Verhalten

### PID-Regelung (Optional)

Aktuell nutzen wir Feedforward (Open-Loop). Für präzisere Regelung:

- Encoder-Polarität korrigieren (Quadratur-Encoder oder Richtungs-Heuristik verbessern)
- PID aktivieren (Kp=13.0, Ki=5.0, Kd=0.01 aus früheren Tests)

---

## Hardware-Übersicht

| Komponente | Spezifikation | Status |
|------------|---------------|--------|
| Seeed XIAO ESP32-S3 | Dual-Core, USB-CDC | ✅ Aktiv |
| Cytron MDD3A | Dual-PWM, 4-16V | ✅ Aktiv |
| JGA25-370 (2×) | 12V DC + Encoder | ✅ Aktiv |
| Raspberry Pi 5 | 8GB, ROS 2 Humble | ✅ Aktiv |
| RPLidar A1 | 360° 2D Lidar | ✅ Erkannt |
| Hailo-8L | AI Accelerator | ⬜ Später |
| IMX296 | Global Shutter | ⬜ Später |
| MPU6050 | IMU (I2C) | ⬜ Später |

---

## Risikomatrix

| Risiko | Wahrscheinlichkeit | Impact | Status |
|--------|-------------------|--------|--------|
| micro-ROS inkompatibel | ~~Hoch~~ | ~~Hoch~~ | ✅ **Gelöst** |
| MDD3A-Ansteuerung | ~~Hoch~~ | ~~Hoch~~ | ✅ **Dual-PWM** |
| PID-Eskalation | ~~Mittel~~ | ~~Mittel~~ | ✅ **Feedforward** |
| Motor-Richtung falsch | ~~Mittel~~ | ~~Mittel~~ | ✅ **PWM getauscht** |
| Failsafe greift zu früh | ~~Mittel~~ | ~~Niedrig~~ | ✅ **2000ms** |
| RPLidar-Treiber | Niedrig | Mittel | 🔜 Phase 3 |
| Nav2-Tuning aufwändig | Hoch | Mittel | Viel Zeit einplanen |

---

## Zeitplan

```
Woche:  1  2  3  4  5  6  7  8
        ════════════════════════════════
Phase 1 ████                             ✅ micro-ROS
Phase 2 ████                             ✅ Docker
Phase 3       ████                       ◄── RPLidar
Phase 4             ████                 EKF
Phase 5                   ████           SLAM
Phase 6                         ████     Nav2
        ════════════════════════════════
```

---

## Checkliste pro Phase

Jede Phase ist erst abgeschlossen, wenn:

- [x] Die definierten Tests bestanden sind
- [x] Der Code committet und dokumentiert ist
- [x] Die Konfigurationsdateien versioniert sind
- [x] Ein kurzes Protokoll die Ergebnisse festhält
- [x] Der nächste Schritt klar ist

**Phase 1:** ✅ Alle Punkte erfüllt (2025-12-20)
**Phase 2:** ✅ Alle Punkte erfüllt

---

## Changelog

### v2.0 (2025-12-20)

- **Phase 1:** micro-ROS statt Serial-Bridge
- **Firmware:** v3.2.0 mit Feedforward
- **Architektur:** Dual-Core FreeRTOS
- **Docker:** Container-basiertes Deployment
- **Phasen:** Reorganisiert (6 statt 7)

### v1.3 (2025-12-12)

- Phase 2 (Odometrie + PID) abgeschlossen
- Serial-Bridge Architektur (Legacy)

---

*Dieser Plan folgt dem Prinzip: Jede Woche ein lauffähiges System. Lieber weniger Features, die funktionieren, als viele Features, die zusammen crashen.*

*Aktualisiert: 2025-12-20 | Firmware: v3.2.0 | Phase 1 abgeschlossen*
