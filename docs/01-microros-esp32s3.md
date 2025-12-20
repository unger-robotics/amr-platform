---
title: "Phase 1 – micro-ROS auf ESP32-S3 (USB-Serial)"
status: "completed"
updated: "2025-12-20"
version: "3.2.0"
source:
  firmware: "firmware/src/main.cpp"
  config: "firmware/include/config.h"
  platformio: "firmware/platformio.ini"
---

# Phase 1: micro-ROS auf ESP32-S3 (USB-Serial)

## Zielbild & Definition of Done

- ESP32-S3 läuft als **micro-ROS Client** über **USB-CDC (Serial)**.
- `/cmd_vel` steuert Motoren über **Cytron MDD3A Dual-PWM**.
- `/odom_raw` wird publiziert (Pose2D) und ist plausibel.
- **Failsafe** stoppt Motoren nach `FAILSAFE_TIMEOUT_MS = 2000`.

**DoD (verifiziert 2025-12-20):**

- [x] Agent verbindet stabil (Reconnect reproduzierbar).
- [x] `/cmd_vel` wirkt (vor/zurück/rotieren).
- [x] `/odom_raw` plausibel (x steigt vorwärts, theta bei Drehung).
- [x] Timeout-Failsafe stoppt deterministisch nach ~2s.
- [x] `/esp32/heartbeat` läuft (~1 Hz).

---

## Testergebnisse (2025-12-20)

| Test | Befehl | Ergebnis | Status |
|------|--------|----------|--------|
| Agent-Verbindung | – | `fd: 3` stabil | ✅ |
| Heartbeat | `ros2 topic echo /esp32/heartbeat` | ~1 Hz | ✅ |
| Vorwärts | `linear.x: 0.15` | Räder drehen vorwärts | ✅ |
| Rückwärts | `linear.x: -0.15` | Räder drehen rückwärts | ✅ |
| Drehen links | `angular.z: 0.5` | Roboter dreht links | ✅ |
| Drehen rechts | `angular.z: -0.5` | Roboter dreht rechts | ✅ |
| Failsafe | Ctrl+C, 2s warten | Motoren stoppen | ✅ |
| Odom | `ros2 topic echo /odom_raw` | x, y, theta plausibel | ✅ |

**Odom-Beispiel nach Testfahrt:**

```yaml
x: 0.899
y: -0.329
theta: 6.09
```

---

## Systemübersicht

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
│                                                             │
│  Shared Memory: Mutex-geschützt                             │
└─────────────────────────────────────────────────────────────┘
                            │
                      USB-CDC (921600 Baud)
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Docker)                                    │
│                                                             │
│  Container: amr_agent                                       │
│    - micro-ros-agent serial --dev /dev/ttyACM0 -b 921600   │
│                                                             │
│  Container: amr_dev                                         │
│    - ROS 2 Humble Workspace                                 │
│    - ros2 topic pub/echo/hz                                 │
└─────────────────────────────────────────────────────────────┘
```

**Topics (verifiziert):**

| Topic | Typ | Richtung | Funktion |
|-------|-----|----------|----------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Geschwindigkeitsbefehle |
| `/odom_raw` | `geometry_msgs/Pose2D` | Pub | Odometrie (x, y, theta) |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | Lebenszeichen |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | LED-Steuerung |

---

## Hardware (Phase 1)

| Komponente | Spezifikation | Rolle |
|------------|---------------|-------|
| Seeed XIAO ESP32-S3 | Dual-Core Xtensa LX7, USB-CDC | micro-ROS Client + Control |
| Cytron MDD3A | Dual-PWM, 4–16 V | Motortreiber |
| JGA25-370 (2×) | 12V DC + Hall-Encoder | Antrieb + Odometrie |
| Raspberry Pi 5 | ROS 2 Humble (Docker) | micro-ROS Agent + Host |

### Pin-Mapping

| Funktion | Pin | Typ | Hinweis |
|----------|-----|-----|---------|
| Motor Left A | D0 | PWM | → PWM_CH 1 (getauscht) |
| Motor Left B | D1 | PWM | → PWM_CH 0 (getauscht) |
| Motor Right A | D2 | PWM | → PWM_CH 3 (getauscht) |
| Motor Right B | D3 | PWM | → PWM_CH 2 (getauscht) |
| Encoder Left A | D6 | IRQ | A-only |
| Encoder Right A | D7 | IRQ | A-only |
| LED/MOSFET | D10 | GPIO | Status |

**Hinweis:** Die PWM-Kanäle wurden getauscht (A↔B), um die korrekte Fahrtrichtung zu erreichen.

---

## Firmware – Parameter (v3.2.0)

### config.h

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `LOOP_RATE_HZ` | 100 | Control-Zyklus (10 ms) |
| `ODOM_PUBLISH_HZ` | 20 | Odom Publish (50 ms) |
| `FAILSAFE_TIMEOUT_MS` | 2000 | Heartbeat-Timeout |
| `MOTOR_PWM_FREQ` | 20000 | 20 kHz (unhörbar) |
| `MOTOR_PWM_BITS` | 8 | 0-255 Auflösung |
| `PWM_DEADZONE` | 35 | Mindest-PWM |
| `WHEEL_DIAMETER` | 0.065 m | Raddurchmesser |
| `WHEEL_BASE` | 0.178 m | Spurbreite |

### PWM-Kanäle (getauscht für korrekte Richtung)

```cpp
#define PWM_CH_LEFT_A  1  // war 0
#define PWM_CH_LEFT_B  0  // war 1
#define PWM_CH_RIGHT_A 3  // war 2
#define PWM_CH_RIGHT_B 2  // war 3
```

### Regelung (Open-Loop mit Feedforward)

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `PID_KP` | 0.0 | Deaktiviert |
| `PID_KI` | 0.0 | Deaktiviert |
| `PID_KD` | 0.0 | Deaktiviert |
| `feedforward_gain` | 2.0 | Direkte Ansteuerung |

**Hinweis:** PID wurde deaktiviert, da die Encoder-Polarität invertiert ist. Feedforward ermöglicht stabile Open-Loop-Steuerung. PID-Tuning kann in Phase 4+ erfolgen, nachdem die Encoder-Richtungsheuristik validiert wurde.

### main.cpp – Feedforward-Berechnung (Zeile ~384)

```cpp
// Feedforward + PID (Feedforward für Open-Loop, PID für Feinkorrektur)
float feedforward_gain = 2.0f;
float pwm_l = feedforward_gain * set_v_l + pid_left.compute(set_v_l, v_enc_l, dt);
float pwm_r = feedforward_gain * set_v_r + pid_right.compute(set_v_r, v_enc_r, dt);

// Begrenzen auf PWM-Bereich
pwm_l = constrain(pwm_l, -1.0f, 1.0f);
pwm_r = constrain(pwm_r, -1.0f, 1.0f);
```

---

## Build/Flash/Monitor (PlatformIO)

### Firmware kompilieren und flashen (Mac)

```bash
cd ~/daten/start/IoT/AMR/amr-platform/firmware
pio run -e seeed_xiao_esp32s3 -t upload
```

### Serial Monitor (Debug)

```bash
pio device monitor -b 921600
```

---

## Docker-Setup (Pi 5)

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

### Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d
docker compose ps
```

### Agent-Logs prüfen

```bash
docker compose logs microros_agent --tail 10
```

**Erwartete Ausgabe:**

```
amr_agent | [timestamp] info | TermiosAgentLinux.cpp | init | running... | fd: 3
```

---

## Smoke-Tests

### 1. In Container gehen

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
```

### 2. Topics prüfen

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

### 3. Motor-Tests (⚠️ Räder aufbocken!)

```bash
# Vorwärts
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10

# Rückwärts (Ctrl+C, dann:)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.15}, angular: {z: 0.0}}" -r 10

# Drehen links
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10

# Drehen rechts
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.5}}" -r 10
```

### 4. Failsafe-Test

1. Motor-Befehl senden (Räder drehen)
2. `Ctrl+C` drücken
3. 2 Sekunden warten
4. **Erwartung:** Motoren stoppen automatisch

### 5. Odometrie prüfen

```bash
ros2 topic echo /odom_raw --once
```

---

## Troubleshooting

| Problem | Ursache | Lösung |
|---------|---------|--------|
| `Serial port not found` | ESP32 nicht angeschlossen | USB-Kabel prüfen, `ls /dev/ttyACM*` |
| Topics fehlen | Agent nicht verbunden | Agent-Logs prüfen, ESP32 Reset |
| Räder drehen falsche Richtung | PWM-Kanäle falsch | A↔B tauschen in config.h |
| Motor reagiert nicht | Feedforward zu niedrig | `feedforward_gain` erhöhen |
| PID eskaliert | Encoder-Polarität invertiert | PID deaktivieren (Kp=0) |
| Failsafe greift nicht | Timeout zu kurz | `FAILSAFE_TIMEOUT_MS` erhöhen |

---

## Bekannte Einschränkungen

1. **Open-Loop-Steuerung:** PID deaktiviert, keine Geschwindigkeitsregelung
2. **Encoder A-only:** Richtung wird aus Soll-Geschwindigkeit abgeleitet
3. **Odom-Rate:** Effektiv ~3-6 Hz durch Serial-Transport

---

## Nächste Schritte

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 1 | micro-ROS ESP32-S3 | ✅ Abgeschlossen |
| Phase 2 | Docker-Infrastruktur | ✅ Vorhanden |
| Phase 3 | RPLidar A1 Integration | 🔜 Bereit (`/dev/ttyUSB0`) |
| Phase 4 | EKF Sensor Fusion | ⏳ |
| Phase 5 | SLAM (slam_toolbox) | ⏳ |
| Phase 6 | Nav2 Autonome Navigation | ⏳ |

---

## Changelog

| Version | Datum | Änderungen |
|---------|-------|------------|
| v1.0 | 2025-12-19 | Initiale Dokumentation |
| v3.1.0 | 2025-12-20 | PID aktiviert, Baudrate 921600 |
| v3.2.0 | 2025-12-20 | PWM-Kanäle getauscht, Feedforward (Gain=2.0), PID deaktiviert, alle Tests bestanden |
