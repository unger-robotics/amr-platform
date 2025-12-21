---
title: "AMR Drive-Base – ESP32-S3 micro-ROS Low-Level Controller"
repo: "amr-drivebase-esp32s3"
status: "active"
version: "v3.1.0 (Firmware) / v2.1.0 (PlatformIO)"
stand: "2025-12-21"
phase: "Phase 3.3 (Odometrie)"
author: "Jan Unger"
---

# AMR Drive-Base – ESP32-S3 micro-ROS Low-Level Controller

Low-Level Firmware für eine Differential-Drive-Base auf **Seeed Studio XIAO ESP32-S3** mit **Cytron MDD3A** (Dual PWM) und **JGA25-370 Hall-Encodern**.
Die Firmware trennt **Echtzeit-Regelung** und **ROS-Kommunikation** strikt (Dual-Core) und stellt eine schlanke micro-ROS Schnittstelle für die High-Level Pipeline (EKF/SLAM/Nav2) bereit.

---

## Ziel

**Ziel der Firmware:** Aus ROS 2 Kommandos (`cmd_vel`) deterministische Motoransteuerung erzeugen und eine robuste, bandbreitenschonende Odometrie (`odom_raw`) bereitstellen – inklusive Safety-Failsafe.

- **Eingang:** `cmd_vel` (`geometry_msgs/msg/Twist`)
  *linear.x* in $\mathrm{m/s}$, *angular.z* in $\mathrm{rad/s}$
- **Ausgang:** `odom_raw` (`geometry_msgs/msg/Pose2D`)
  *x*, *y* in $\mathrm{m}$, *theta* in $\mathrm{rad}$
- **Diagnose:** `esp32/heartbeat` (`std_msgs/msg/Int32`), `esp32/led_cmd` (`std_msgs/msg/Bool`)

---

## Regeln (Design- und Safety-Regelwerk)

### 1) Architektur-Regel (Dual-Core Trennung)

- **Core 0 (Pro CPU):** harte Echtzeit
  **$100\,\mathrm{Hz}$** Control-Loop, Encoder/Odometrie-Integration, Safety
- **Core 1 (App CPU):** Kommunikation
  micro-ROS Executor (DDS/Serial), Publish/Subscribe, Heartbeat
- **Datenaustausch:** ausschließlich über **Shared Memory + FreeRTOS Mutex** (kein “Nebenher-Zugriff”).

### 2) Standards (Einheiten & Frames)

- **REP-103:** SI-Einheiten (m, s, rad)
- **REP-105:** Frame-Konzept (High-Level Mapping in ROS; diese Firmware publiziert bewusst schlank als `Pose2D`)

### 3) Safety-Regeln (Fail-Safe)

- **Dead-Man / Watchdog:** Wenn länger als `FAILSAFE_TIMEOUT_MS = 2000` keine gültige `cmd_vel` empfangen wurde → **Motorstop**.
- **Deterministisches Timing:** Regelung mit `vTaskDelayUntil` (Jitter-reduziert).
- **Bandbreitenregel:** Odometrie-Publish auf **$20\,\mathrm{Hz}$** limitiert.

---

## Schnittstellen (ROS 2 Topics)

| Topic | Richtung | Typ | Zweck |
|---|---:|---|---|
| `cmd_vel` | Sub | `geometry_msgs/msg/Twist` | Sollwerte: `linear.x`, `angular.z` |
| `odom_raw` | Pub | `geometry_msgs/msg/Pose2D` | Roh-Odometrie (x,y,theta) |
| `esp32/heartbeat` | Pub | `std_msgs/msg/Int32` | Lebenszeichen / Debug-Zähler |
| `esp32/led_cmd` | Sub | `std_msgs/msg/Bool` | LED/MOSFET schalten (Status/Scheinwerfer) |

**Node-Name:** `esp32_dual_core`

---

## Hardware

### Pinout (XIAO ESP32-S3)

| Funktion | Pin | Hinweis |
|---|---|---|
| Motor Left A/B | `D0` / `D1` | MDD3A M1A/M1B (Dual PWM: Vorwärts/Rückwärts) |
| Motor Right A/B | `D2` / `D3` | MDD3A M2A/M2B |
| Encoder Left | `D6` | Interrupt (Single-Channel Hall) |
| Encoder Right | `D7` | Interrupt (Single-Channel Hall) |
| LED/MOSFET | `D10` | Low-Side Switch (IRLZ24N) |
| I²C SDA/SCL | `D4` / `D5` | MPU6050 (später) |
| Servo Pan/Tilt | `D8` / `D9` | später |

### Kinematik (Default-Werte in `config.h`)

- Raddurchmesser: $0{,}065\,\mathrm{m}$
- Spurbreite (Wheel Base): $0{,}178\,\mathrm{m}$
- Encoder-Kalibrierung (10-Umdrehungen-Test, 2025-12-12):
  Links $374{,}3$, rechts $373{,}6$ Ticks/Umdrehung

**Wichtig (Limitation):** Single-Channel Encoder liefern nur Ticks, keine Richtung. Die Firmware nutzt dafür eine Richtungs-Heuristik aus dem Sollwert (für saubere EKF/SLAM später ggf. auf Quadratur-Encoder upgraden).

---

## Repository-Struktur (PlatformIO)

```text
firmware/
├─ platformio.ini
├─ include/
│  └─ config.h
└─ src/
   └─ main.cpp
```

---

## Build & Flash

### Voraussetzungen

- PlatformIO (VS Code oder CLI)
- USB-Verbindung zur XIAO ESP32-S3

### Kommandos (CLI)

```bash
# Build
pio run -e seeed_xiao_esp32s3

# Flash
pio run -e seeed_xiao_esp32s3 -t upload

# Serieller Monitor (Firmware nutzt 921600 Baud)
pio device monitor -b 921600
```

`platformio.ini` setzt u. a.:

- Upload-Speed: `921600`
- USB CDC on boot (für stabile Serial/micro-ROS Verbindung)

---

## Betrieb (micro-ROS Agent + Smoke Tests)

### 1) micro-ROS Agent starten (Host)

Beispiel (Linux):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600
```

macOS typischer Port:

- `/dev/cu.usbmodem*`

### 2) Topics prüfen

```bash
ros2 node list
ros2 topic list
ros2 topic echo /esp32/heartbeat
ros2 topic echo /odom_raw
```

### 3) Bewegung testen (`cmd_vel`)

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.20}, angular: {z: 0.0}}"
```

### 4) LED/MOSFET testen

```bash
ros2 topic pub /esp32/led_cmd std_msgs/msg/Bool "{data: true}"
```

---

## Wichtige Parameter (Kurzüberblick)

| Parameter             | Default | Ort        | Bedeutung                |
| --------------------- | ------: | ---------- | ------------------------ |
| `LOOP_RATE_HZ`        |   `100` | `config.h` | Regel-Loop Core 0        |
| `ODOM_PUBLISH_HZ`     |    `20` | `config.h` | Publish-Rate Core 1      |
| `FAILSAFE_TIMEOUT_MS` |  `2000` | `config.h` | Motorstop ohne `cmd_vel` |
| `MAX_LINEAR_SPEED`    |   `0.5` | `config.h` | $\mathrm{m/s}$ Limit     |
| `MAX_ANGULAR_SPEED`   |   `2.0` | `config.h` | $\mathrm{rad/s}$ Limit   |

---

## Projektstand (Roadmap – 6 Phasen)

| Phase | Beschreibung                        | Status          |
| ----- | ----------------------------------- | --------------- |
| 1     | micro-ROS auf ESP32-S3 (USB-Serial) | ✅ abgeschlossen |
| 2     | Docker-Infrastruktur                | ✅ vorhanden     |
| 3     | RPLidar A1 Integration              | ◄── **aktuell** |
| 4     | EKF Sensor Fusion                   | ⬜               |
| 5     | SLAM (slam_toolbox)                 | ⬜               |
| 6     | Nav2 Autonome Navigation            | ⬜               |

---

## Lizenz / Hinweise

- Firmware ist „Safety-first“ ausgelegt (Failsafe aktiv).
- Vor echtem Fahrbetrieb: mechanische Sicherheit (Not-Aus), Strombegrenzung, Testbock, Freilauf prüfen.
