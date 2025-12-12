# AMR Entwicklerdokumentation

> **Stand:** 2025-12-12 | **Phase:** 2 ✅ ABGESCHLOSSEN | **Firmware:** v0.5.0-pid

---

## 1. Aktueller Projektstatus

### 1.1 Was funktioniert ✅

| Komponente | Status | Validiert |
|------------|--------|-----------|
| Raspberry Pi 5 | ✅ | OS, Docker, SSH |
| Hailo-8L | ✅ | 31 FPS, HailoRT 4.23.0 |
| RPLIDAR A1 | ✅ | 7.5 Hz, TF korrekt |
| Kamera IMX296 | ✅ | libcamera 0.3+ |
| ESP32-S3 XIAO | ✅ | USB-CDC erkannt |
| Motor Forward | ✅ | Beide Räder vorwärts |
| Motor Backward | ✅ | Beide Räder rückwärts |
| Motor Turn Left | ✅ | Differential Drive |
| Motor Turn Right | ✅ | Differential Drive |
| LED-Status (D10) | ✅ | Breathing + Blink |
| PWM (20 kHz) | ✅ | Unhörbar |
| Failsafe (500ms) | ✅ | Motoren stoppen bei Timeout |
| Deadzone-Kompensation | ✅ | Auch kleine Geschwindigkeiten |
| Git-Workflow | ✅ | Mac → GitHub → Pi |
| Docker-Stack | ✅ | perception + serial_bridge |
| Serial-Bridge | ✅ | ROS 2 /cmd_vel → ESP32 |
| Teleop | ✅ | Tastatursteuerung |
| **Encoder-Kalibrierung** | ✅ | 374.3 / 373.6 Ticks/Rev |
| **Odometrie** | ✅ | x, y, theta berechnet |
| **PID-Regelung** | ✅ | Kp=13, Ki=5, Kd=0.01 |
| **ROS 2 /odom** | ✅ | nav_msgs/Odometry |
| **TF odom→base_link** | ✅ | Broadcast aktiv |

### 1.2 Was blockiert ist ⚠️

| Problem | Ursache | Workaround |
|---------|---------|------------|
| micro-ROS Build | Python 3.13 inkompatibel | ✅ Serial-Bridge |

### 1.3 Offene Punkte (Phase 3)

- [ ] RPLIDAR in Docker integrieren
- [ ] SLAM Toolbox konfigurieren
- [ ] Erste Karte erstellen

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
| LED-Strip MOSFET | D10 | CH 4 | IRLZ24N |

### 2.2 Cytron MDD3A - Dual-PWM Steuerung

**WICHTIG:** Der MDD3A verwendet **Dual-PWM**, NICHT PWM+DIR!

| M1A | M1B | Ergebnis |
|-----|-----|----------|
| PWM | 0 | Vorwärts |
| 0 | PWM | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |
| PWM | PWM | Active Brake |

### 2.3 Encoder-Kalibrierung (Phase 2)

| Parameter | Theoretisch | Kalibriert | Methode |
|-----------|-------------|------------|---------|
| Ticks/Rev Links | 390.5 | **374.3** | 10-Umdrehungen-Test |
| Ticks/Rev Rechts | 390.5 | **373.6** | 10-Umdrehungen-Test |
| Abweichung | - | -4.2% | Getriebespiel |

---

## 3. Software-Architektur

### 3.1 Serial-Bridge mit Odometrie (Phase 2)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                   Serial-Bridge Architektur (v0.5.0)                    │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   ROS 2 (Docker)              USB-Serial              ESP32-S3          │
│  ┌──────────────┐            ┌────────┐            ┌──────────────┐    │
│  │ /cmd_vel     │───────────▶│ Bridge │───────────▶│ PID-Regler   │    │
│  │ Twist msg    │            │ Node   │  V:0.2,    │ pro Rad      │    │
│  └──────────────┘            │ Python │  W:0.0\n   └──────────────┘    │
│                              └────────┘                  │              │
│  ┌──────────────┐                 ▲                      ▼              │
│  │ /odom        │◀────────────────│            ┌──────────────┐        │
│  │ Odometry msg │   ODOM:l,r,     │            │ Encoder ISR  │        │
│  └──────────────┘   x,y,theta     │            │ Odometrie    │        │
│                                   │            └──────────────┘        │
│  ┌──────────────┐                 │                                    │
│  │ TF: odom →   │◀────────────────┘                                    │
│  │   base_link  │                                                      │
│  └──────────────┘                                                      │
│                               /dev/ttyACM0                             │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Serial-Protokoll (v0.5.0)

| Richtung | Format | Beispiel |
|----------|--------|----------|
| Host → ESP32 | `V:<m/s>,W:<rad/s>\n` | `V:0.20,W:0.00\n` |
| Host → ESP32 | `RESET_ODOM\n` | Odometrie zurücksetzen |
| Host → ESP32 | `PID:<Kp>,<Ki>,<Kd>\n` | `PID:13.0,5.0,0.01\n` |
| Host → ESP32 | `DEBUG:ON\n` / `DEBUG:OFF\n` | VEL-Nachrichten |
| ESP32 → Host | `OK:<v>,<w>` | Bestätigung |
| ESP32 → Host | `ODOM:<l>,<r>,<x>,<y>,<th>\n` | Odometrie (50 Hz) |
| ESP32 → Host | `VEL:<vl>,<vr>,<pwml>,<pwmr>\n` | Debug (10 Hz) |
| ESP32 → Host | `FAILSAFE:TIMEOUT` | Nach 500ms ohne Befehl |

### 3.3 Docker Container

| Container | Image | Funktion |
|-----------|-------|----------|
| `amr_perception` | amr_perception:jazzy | ROS 2 Stack, Teleop |
| `amr_serial_bridge` | ros:jazzy-ros-base | Serial-Bridge Node |

---

## 4. Projekt-Struktur

```
amr-platform/                    # GitHub: ju1-eu/amr-platform
├── firmware/                    # micro-ROS Firmware (nicht verwendet)
├── firmware_serial/             # Serial-Bridge Firmware ✅
│   ├── platformio.ini
│   ├── include/config.h         # PID: Kp=13, Ki=5, Kd=0.01
│   └── src/main.cpp             # v0.5.0-pid
├── firmware_test/               # Hardware-Test Firmware
├── docker/
│   ├── docker-compose.yml
│   └── perception/
├── ros2_ws/
│   └── src/
│       ├── amr_description/     # URDF
│       ├── amr_bringup/
│       └── amr_serial_bridge/   # ROS 2 Serial Bridge ✅
│           ├── amr_serial_bridge/
│           │   └── serial_bridge.py  # v0.4.0-odom
│           ├── launch/
│           ├── package.xml
│           └── setup.py
└── scripts/
    └── tune_pid.sh              # PID-Tuning Script
```

---

## 5. Validierte Test-Befehle

### 5.1 ESP32 Firmware flashen (auf Mac)

```bash
cd /Users/jan/daten/start/IoT/AMR/amr-platform/firmware_serial
pio run --target upload
pio device monitor
```

**Erwartete Ausgabe:**

```
AMR-ESP32 v0.5.0-pid ready
PID: Kp=13.00 Ki=5.00 Kd=0.01
Commands: V:v,W:w | RESET_ODOM | PID:Kp,Ki,Kd | DEBUG:ON/OFF
```

### 5.2 PID-Tuning (auf Pi)

```bash
ssh pi@rover
cd ~/amr-platform/docker && docker compose down
~/tune_pid.sh
```

**Befehle im Script:**

- `test` – 1m Geradeaus-Test
- `p 13.0` – Kp setzen
- `i 5.0` – Ki setzen
- `debug` – Velocity-Ausgabe

### 5.3 Docker-Stack (auf Pi)

```bash
ssh pi@rover
cd ~/amr-platform/docker
docker compose up -d
docker compose logs -f serial_bridge
```

### 5.4 Teleop-Test

```bash
docker exec -it amr_perception bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 6. Phase 2 Abschluss-Protokoll

### 6.1 PID-Tuning Ergebnisse

| Parameter | Startwert | Endwert | Methode |
|-----------|-----------|---------|---------|
| Kp | 2.0 | **13.0** | Inkrementell erhöht |
| Ki | 0.5 | **5.0** | Stationären Fehler eliminiert |
| Kd | 0.01 | **0.01** | Beibehalten (kein Überschwingen) |

### 6.2 Bodentest-Ergebnisse (1m @ 0.2 m/s)

| Metrik | Open-Loop | Mit PID | Verbesserung |
|--------|-----------|---------|--------------|
| Distanz x | 1.158 m | 0.984 m | Fehler: 16% → 1.6% |
| Drift y | 14 cm | 0.5 cm | **28× besser** |
| Encoder L/R | 2381/2470 | 1802/1802 | **Synchron** |

### 6.3 Lessons Learned

1. **Encoder-Kalibrierung essentiell** – Theoretische Werte (390.5) weichen 4% ab
2. **PID-Tuning iterativ** – Kp zuerst, dann Ki für stationären Fehler
3. **Heartbeat für Tests** – Failsafe (500ms) erfordert kontinuierliche Befehle
4. **Bodentest vs. aufgebockt** – Schlupf beeinflusst Ergebnisse

---

## 7. Referenzen

| Dokument | Beschreibung |
|----------|--------------|
| `AMR_Implementierungsplan.md` | Phasen 0–6 |
| `Industriestandards-AMR.md` | REP-103, REP-105, Safety |
| `06-git-workflow.md` | Git Mac ↔ Pi |
| `ros2_ws/src/amr_serial_bridge/SETUP.md` | Serial-Bridge Setup |

---

## 8. Changelog

| Datum | Version | Änderung |
|-------|---------|----------|
| 2025-12-12 | v0.5.0-pid | **Phase 2 abgeschlossen** |
| 2025-12-12 | v0.5.0-pid | PID-Tuning: Kp=13, Ki=5, Kd=0.01 |
| 2025-12-12 | v0.4.0-odom | Odometrie + ROS 2 /odom Publisher |
| 2025-12-12 | v0.4.0-odom | Encoder-Kalibrierung: 374.3/373.6 |
| 2025-12-12 | v0.3.0-serial | Phase 1 abgeschlossen |

---

## 9. Nächste Schritte: Phase 3

1. **RPLIDAR A1** in Docker Container integrieren
2. **slam_toolbox** konfigurieren
3. **Testraum kartieren** – erste Karte erstellen
4. **Karte speichern** – PGM + YAML

---

*Dokumentation erstellt: 2025-12-12 | Autor: Jan Unger | FH Aachen*
