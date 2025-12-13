# AMR Entwicklerdokumentation

> **Stand:** 2025-12-13 | **Phase:** 3.2 ✅ Motor-Control | **Firmware:** v2.0.0-microros

---

## 1. Aktueller Projektstatus

### 1.1 Architektur-Wechsel (2025-12-13)

| Aspekt | Alt (Serial-Bridge) | Neu (micro-ROS) |
|--------|---------------------|-----------------|
| Protokoll | Custom Text `V:0.2,W:0.0\n` | DDS/XRCE (Standard) |
| ROS 2 Integration | Python Bridge-Node | Native Topics |
| Agent | Docker Container | **systemd Service** |
| Distribution | Jazzy | **Humble** |

### 1.2 Was funktioniert ✅

| Komponente | Status | Validiert |
|------------|--------|-----------|
| Raspberry Pi 5 | ✅ | OS, Docker, SSH |
| Hailo-8L | ✅ | 31 FPS, HailoRT 4.23.0 |
| RPLIDAR A1 | ✅ | 7.5 Hz, TF korrekt |
| Kamera IMX296 | ✅ | libcamera 0.3+ |
| ESP32-S3 XIAO | ✅ | USB-CDC erkannt |
| **micro-ROS Agent** | ✅ | systemd Service, Autostart |
| **micro-ROS Firmware** | ✅ | v2.0.0, Humble |
| **/cmd_vel Subscriber** | ✅ | geometry_msgs/Twist |
| **/esp32/heartbeat** | ✅ | std_msgs/Int32, 1 Hz |
| **/esp32/led_cmd** | ✅ | std_msgs/Bool |
| **Motor Forward** | ✅ | Beide Räder vorwärts |
| **Motor Backward** | ✅ | Beide Räder rückwärts |
| **Motor Turn** | ✅ | Differential Drive |
| **Failsafe** | ✅ | 500ms Timeout |
| **Teleop** | ✅ | Tastatursteuerung |

### 1.3 Offene Punkte (Phase 3.3+)

- [ ] Odometrie (`/odom`) Publisher
- [ ] TF-Broadcast (`odom` → `base_link`)
- [ ] IMU Integration (optional)
- [ ] SLAM Integration

---

## 2. Hardware-Konfiguration

### 2.1 Pin-Belegung ESP32-S3 XIAO

| Funktion | Pin | PWM-Kanal | Notiz |
|----------|-----|-----------|-------|
| Motor Links A (vorwärts) | D0 | CH 0 | Cytron M1A |
| Motor Links B (rückwärts) | D1 | CH 1 | Cytron M1B |
| Motor Rechts A (vorwärts) | D2 | CH 2 | Cytron M2A |
| Motor Rechts B (rückwärts) | D3 | CH 3 | Cytron M2B |
| I2C SDA (IMU) | D4 | - | MPU6050 |
| I2C SCL (IMU) | D5 | - | MPU6050 |
| Encoder Links | D6 | - | Interrupt |
| Encoder Rechts | D7 | - | Interrupt |
| LED-Strip MOSFET | D10 | - | IRLZ24N |
| Onboard LED | GPIO 21 | - | Status (active LOW) |

### 2.2 Cytron MDD3A - Dual-PWM Steuerung

| M1A | M1B | Ergebnis |
|-----|-----|----------|
| PWM | 0 | Vorwärts |
| 0 | PWM | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |
| PWM | PWM | Active Brake |

---

## 3. Software-Architektur (micro-ROS)

### 3.1 Systemübersicht

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    micro-ROS Architektur (v2.0.0)                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ROS 2 (Humble)           USB-Serial           ESP32-S3 (micro-ROS)   │
│  ┌──────────────┐         ┌─────────┐          ┌──────────────────┐    │
│  │ /cmd_vel     │────────▶│ Agent   │─────────▶│ Twist Subscriber │    │
│  │ Twist msg    │         │ (DDS)   │          │ → Diff Drive     │    │
│  └──────────────┘         │         │          │ → PWM Output     │    │
│                           │ systemd │          └──────────────────┘    │
│  ┌──────────────┐         │ Service │          ┌──────────────────┐    │
│  │ /esp32/      │◀────────│         │◀─────────│ Heartbeat Pub    │    │
│  │ heartbeat    │         │         │          │ (1 Hz)           │    │
│  └──────────────┘         └─────────┘          └──────────────────┘    │
│                                                                         │
│                           /dev/ttyACM0                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 ROS 2 Topics

| Topic | Typ | Richtung | QoS | Beschreibung |
|-------|-----|----------|-----|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | → ESP32 | Best Effort | Geschwindigkeitsbefehl |
| `/esp32/heartbeat` | `std_msgs/Int32` | ← ESP32 | Best Effort | Watchdog (1 Hz) |
| `/esp32/led_cmd` | `std_msgs/Bool` | → ESP32 | Reliable | MOSFET-Steuerung |

### 3.3 Failsafe-Mechanismus

- **Timeout:** 500ms ohne `/cmd_vel` → Motoren stoppen
- **Implementierung:** `last_cmd_time` Tracking in Firmware
- **Status-LED:** Blinkt bei Agent-Suche, dauerhaft an bei Verbindung

---

## 4. Betriebsanleitung

### 4.1 Systemstart

Der micro-ROS Agent startet automatisch bei Boot:

```bash
# Status prüfen
sudo systemctl status microros-agent

# Logs anzeigen
sudo journalctl -u microros-agent -f

# Manuell neustarten
sudo systemctl restart microros-agent
```

### 4.2 Topics prüfen

```bash
docker run -it --rm --net=host ros:humble ros2 topic list
```

Erwartete Ausgabe:

```
/cmd_vel
/esp32/heartbeat
/esp32/led_cmd
/parameter_events
/rosout
```

### 4.3 Teleop starten

```bash
docker run -it --rm --net=host ros:humble bash -c \
  "apt-get update && apt-get install -y ros-humble-teleop-twist-keyboard && \
   ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

| Taste | Aktion |
|-------|--------|
| `i` | Vorwärts |
| `,` | Rückwärts |
| `j` | Links drehen |
| `l` | Rechts drehen |
| `k` | Stopp |
| `q/z` | Speed ±10% |

### 4.4 Direkte Befehle

```bash
# Vorwärts (0.2 m/s)
docker run -it --rm --net=host ros:humble \
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10

# Stopp
docker run -it --rm --net=host ros:humble \
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# LED ein
docker run -it --rm --net=host ros:humble \
  ros2 topic pub --once /esp32/led_cmd std_msgs/msg/Bool "{data: true}"
```

---

## 5. Entwicklungsumgebung

### 5.1 ESP32 Firmware flashen (Mac)

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform/esp32_microros_test
pio run -t upload
```

### 5.2 micro-ROS Cache löschen (bei Problemen)

```bash
rm -rf .pio/libdeps/seeed_xiao_esp32s3/micro_ros_platformio/libmicroros
pio run
```

### 5.3 Distribution Sync

| Komponente | Distribution |
|------------|--------------|
| ESP32 Firmware | **Humble** |
| Docker Agent | **Humble** |
| ROS 2 Host | Humble oder Jazzy |

---

## 6. Projekt-Struktur

```
amr-platform/
├── esp32_microros_test/     # ◄── AKTIV (micro-ROS v2.0.0)
│   ├── include/config.h     # Hardware-Konfiguration
│   ├── src/main.cpp         # Motor-Control Firmware
│   ├── platformio.ini       # Build-Konfiguration
│   └── README.md
├── firmware_serial/          # ARCHIV (Serial-Bridge v0.5.0)
├── docker/
│   └── docker-compose.yml   # Perception Stack
├── scripts/
│   ├── setup_microros_service.sh
│   └── microros-agent.service
├── ros2_ws/
└── docs/
```

---

## 7. Bekannte Warnungen

| Warnung | Ursache | Auswirkung |
|---------|---------|------------|
| `sequence size exceeds remaining buffer` | Serial QoS Artefakt | Funktional unkritisch |
| `ExecStartPre... status=1/FAILURE` | Container existiert nicht | Normal beim ersten Start |

---

## 8. Changelog

| Datum | Version | Änderung |
|-------|---------|----------|
| **2025-12-13** | **v2.0.0** | **micro-ROS Motor-Control validiert** |
| 2025-12-13 | v2.0.0 | Architektur-Wechsel: Serial-Bridge → micro-ROS |
| 2025-12-13 | v2.0.0 | systemd Service für Agent |
| 2025-12-13 | v2.0.0 | /cmd_vel → Differential Drive |
| 2025-12-13 | v1.4.0 | micro-ROS Kommunikation validiert |
| 2025-12-12 | v0.5.0-pid | Phase 2: PID-Regelung |
| 2025-12-12 | v0.4.0-odom | Phase 2: Odometrie |
| 2025-12-12 | v0.3.0-serial | Phase 1: Serial-Bridge |

---

## 9. Nächste Schritte

| Priorität | Feature | Status |
|-----------|---------|--------|
| 1 | ~~Agent Service~~ | ✅ Phase 3.1 |
| 2 | ~~Motor-Control (/cmd_vel)~~ | ✅ Phase 3.2 |
| 3 | Odometrie (/odom) | ⬜ Phase 3.3 |
| 4 | IMU Integration | ⬜ Optional |
| 5 | SLAM | ⬜ Phase 4 |

---

*Dokumentation erstellt: 2025-12-13 | micro-ROS v2.0.0 | Phase 3.2 abgeschlossen*
