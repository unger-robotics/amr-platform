# Systemdokumentation: AMR Low-Level Controller

**Version:** 3.2.0 | **Datum:** 20.12.2025 | **Status:** ✅ Phase 1 abgeschlossen

---

## 1. Architektur-Übersicht

Das System implementiert eine **Hybrid-Echtzeit-Architektur**. Harte Echtzeit-Anforderungen (Motorregelung) werden strikt von Kommunikations-Aufgaben (micro-ROS) getrennt durch Dual-Core-Nutzung des ESP32-S3.

### 1.1 Datenfluss-Diagramm

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 (micro-ROS Client) - Firmware v3.2.0              │
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

---

## 2. Firmware-Architektur (Dual-Core)

Die Firmware nutzt das **FreeRTOS** Betriebssystem des ESP32-S3, um zwei parallele Tasks auf den physischen CPU-Kernen auszuführen.

### 2.1 Core 0: Das "Rückenmark" (Hard Real-Time)

| Eigenschaft | Wert |
|-------------|------|
| Task Name | `controlTask` |
| Frequenz | 100 Hz (Deterministisch via `vTaskDelayUntil`) |
| Priorität | Hoch (`configMAX_PRIORITIES - 1`) |

**Aufgaben:**

1. Encoder-Interrupts auslesen (Atomar)
2. Odometrie integrieren (x, y, θ)
3. **Feedforward-Steuerung** berechnen (Gain=2.0)
4. **Safety-Check:** Heartbeat-Timeout (2000ms) → Not-Halt

### 2.2 Core 1: Das "Gehirn" (Communication)

| Eigenschaft | Wert |
|-------------|------|
| Task | `loop()` (Arduino Standard) |
| Odom Publish | 20 Hz |
| Heartbeat | 1 Hz |

**Aufgaben:**

1. micro-ROS Executor Spin (Datenempfang/Versand)
2. Serialisierung der DDS-Nachrichten
3. I2C-Kommunikation (Zukunft: IMU)

**Datenaustausch:** Über `SharedData` Struct, geschützt durch **Mutex** (Semaphore).

### 2.3 Steuerungslogik

```cpp
// Feedforward + PID (PID aktuell deaktiviert)
float feedforward_gain = 2.0f;
float pwm_l = feedforward_gain * set_v_l + pid_left.compute(set_v_l, v_enc_l, dt);
float pwm_r = feedforward_gain * set_v_r + pid_right.compute(set_v_r, v_enc_r, dt);

// Begrenzen auf PWM-Bereich
pwm_l = constrain(pwm_l, -1.0f, 1.0f);
pwm_r = constrain(pwm_r, -1.0f, 1.0f);
```

**Hinweis:** PID ist deaktiviert (Kp=Ki=Kd=0), da die Encoder-Polarität invertiert ist. Feedforward ermöglicht stabile Open-Loop-Steuerung.

---

## 3. ROS 2 Schnittstelle (API)

### 3.1 Topics

| Topic | Typ | Richtung | Frequenz | QoS | Beschreibung |
|-------|-----|----------|----------|-----|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | - | Reliable | Geschwindigkeitsbefehle |
| `/odom_raw` | `geometry_msgs/Pose2D` | Pub | 20 Hz | Best Effort | Odometrie (x, y, theta) |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | 1 Hz | Best Effort | Lebenszeichen |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | - | Reliable | LED/MOSFET Steuerung |

### 3.2 Nachrichtenformate

**cmd_vel (Input):**

```yaml
linear:
  x: 0.15    # [m/s] Vorwärts (+) / Rückwärts (-)
  y: 0.0     # Nicht verwendet
  z: 0.0     # Nicht verwendet
angular:
  x: 0.0     # Nicht verwendet
  y: 0.0     # Nicht verwendet
  z: 0.5     # [rad/s] Links (+) / Rechts (-)
```

**odom_raw (Output):**

```yaml
x: 0.899     # [m] Position X
y: -0.329    # [m] Position Y
theta: 6.09  # [rad] Orientierung
```

---

## 4. Konfiguration & Parameter

### 4.1 config.h

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `LOOP_RATE_HZ` | 100 | Control-Zyklus (10 ms) |
| `ODOM_PUBLISH_HZ` | 20 | Odom Publish (50 ms) |
| `FAILSAFE_TIMEOUT_MS` | **2000** | Heartbeat-Timeout |
| `MOTOR_PWM_FREQ` | 20000 | 20 kHz (unhörbar) |
| `MOTOR_PWM_BITS` | 8 | 0-255 Auflösung |
| `PWM_DEADZONE` | 35 | Mindest-PWM |
| `WHEEL_DIAMETER` | 0.065 m | Raddurchmesser |
| `WHEEL_BASE` | 0.178 m | Spurbreite |

### 4.2 PWM-Kanäle (getauscht für korrekte Richtung)

```cpp
#define PWM_CH_LEFT_A  1  // war 0
#define PWM_CH_LEFT_B  0  // war 1
#define PWM_CH_RIGHT_A 3  // war 2
#define PWM_CH_RIGHT_B 2  // war 3
```

### 4.3 Regelung

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `PID_KP` | 0.0 | Deaktiviert |
| `PID_KI` | 0.0 | Deaktiviert |
| `PID_KD` | 0.0 | Deaktiviert |
| `feedforward_gain` | 2.0 | Direkte Ansteuerung |

### 4.4 Hardware Abstraction (HAL)

| Pin | Funktion | Modus | Hardware |
|-----|----------|-------|----------|
| D0 | Motor Left A | PWM → CH 1 | Cytron MDD3A |
| D1 | Motor Left B | PWM → CH 0 | Cytron MDD3A |
| D2 | Motor Right A | PWM → CH 3 | Cytron MDD3A |
| D3 | Motor Right B | PWM → CH 2 | Cytron MDD3A |
| D6 | Encoder Left | ISR (Rising) | JGA25-370 |
| D7 | Encoder Right | ISR (Rising) | JGA25-370 |
| D10 | LED/MOSFET | Digital Out | IRLZ24N |
| D4, D5 | I2C | Wire | *Reserviert (MPU6050)* |
| D8, D9 | Servo | PWM | *Reserviert (Kamera)* |

---

## 5. Inbetriebnahme

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
cd ~/amr-platform/docker
docker compose restart microros_agent
sleep 5
docker compose logs microros_agent --tail 5
```

### 5.3 Verifikation

**Schritt 1: Topics prüfen**

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
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

**Schritt 2: Heartbeat prüfen**

```bash
ros2 topic echo /esp32/heartbeat
```

**Erwartung:** Counter incrementiert ~1×/s

**Schritt 3: Odometrie prüfen**

```bash
ros2 topic echo /odom_raw --once
```

**Schritt 4: Motor-Test (⚠️ Räder aufbocken!)**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

---

## 6. Testergebnisse (2025-12-20)

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

---

## 7. Known Issues & Lösungen

| Symptom | Ursache | Lösung |
|---------|---------|--------|
| **Roboter "ruckelt"** | Failsafe greift ein | `FAILSAFE_TIMEOUT_MS` erhöhen (aktuell 2000ms) |
| **Keine Odom-Daten** | QoS Mismatch | Best Effort QoS nutzen |
| **Motor reagiert nicht** | Feedforward zu niedrig | `feedforward_gain` erhöhen |
| **PID eskaliert** | Encoder-Polarität invertiert | PID deaktivieren (Kp=0) |
| **Räder drehen falsch** | PWM-Kanäle | A↔B tauschen in config.h |
| **Topics fehlen** | Agent nicht verbunden | `docker compose restart microros_agent` |

---

## 8. Bekannte Einschränkungen

1. **Open-Loop-Steuerung:** PID deaktiviert, keine Geschwindigkeitsregelung
2. **Encoder A-only:** Richtung wird aus Soll-Geschwindigkeit abgeleitet
3. **Odom-Rate:** Effektiv ~3-6 Hz durch Serial-Transport

---

## 9. Projektstruktur

```
amr-platform/
├── firmware/                 # ◄── v3.2.0 (Dual-Core)
│   ├── src/main.cpp          # FreeRTOS + micro-ROS + Feedforward
│   ├── include/config.h      # PWM-Kanäle getauscht
│   └── platformio.ini
├── docker/
│   └── docker-compose.yml    # amr_agent + amr_dev
├── ros2_ws/
│   └── src/
└── docs/
    ├── 01-microros-esp32s3.md
    ├── phase1-befehle.md
    ├── todo-liste.md
    └── systemdokumentation.md
```

---

## 10. Changelog

### v3.2.0 (20.12.2025) – Phase 1 Abschluss

- **Motor-Richtung:** PWM-Kanäle getauscht (A↔B)
- **Steuerung:** Feedforward (Gain=2.0) statt PID
- **PID:** Deaktiviert (Kp=Ki=Kd=0) wegen Encoder-Polarität
- **Failsafe:** Timeout auf 2000ms erhöht
- **Tests:** Alle Richtungen validiert

### v3.1.0 (20.12.2025)

- **Baudrate:** 921600 (war 115200)
- **PID:** Aktiviert (Kp=1.0)
- **Problem:** PID-Eskalation durch Encoder-Polarität

### v3.0.0 (14.12.2025) – Major Release

- **Architektur:** Wechsel auf Dual-Core (App/Pro CPU Trennung)
- **RTOS:** Einführung von FreeRTOS Tasks und Mutex-Synchronisation
- **Daten:** Optimierung auf `Pose2D` (Bandbreitenersparnis ~60%)
- **Hardware:** Vollständige Initialisierung aller Pins

### v2.2.2 (13.12.2025) – Legacy

- Single-Loop Architektur
- 3 separate Float-Topics (veraltet)

---

## 11. Nächste Schritte

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 1 | micro-ROS ESP32-S3 | ✅ Abgeschlossen |
| Phase 2 | Docker-Infrastruktur | ✅ Vorhanden |
| Phase 3 | RPLidar A1 Integration | 🔜 Bereit |
| Phase 4 | EKF Sensor Fusion | ⏳ |
| Phase 5 | SLAM (slam_toolbox) | ⏳ |
| Phase 6 | Nav2 Autonome Navigation | ⏳ |
