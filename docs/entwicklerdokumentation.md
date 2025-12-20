# Entwicklerdokumentation: AMR Low-Level Controller

**Version:** 3.2.0 | **Stand:** 20.12.2025 | **Status:** ✅ Phase 1 abgeschlossen

---

## 1. Architektur-Design

Das System folgt einer **Hybrid-Echtzeit-Architektur**. Physikalische Regelung wird strikt von der Datenkommunikation getrennt durch Dual-Core-Nutzung des ESP32-S3.

### 1.1 Dual-Core Aufteilung (ESP32-S3)

Der ESP32-S3 verfügt über zwei Kerne. Wir nutzen **FreeRTOS**, um Aufgaben basierend auf ihrer Zeitkritikalität zuzuweisen.

```
┌─────────────────────────────────────────────────────────────┐
│  ESP32-S3 Firmware v3.2.0                                   │
│                                                             │
│  ┌─────────────────────┐    ┌─────────────────────┐        │
│  │  Core 0 (Pro CPU)   │    │  Core 1 (App CPU)   │        │
│  │  Harte Echtzeit     │    │  Kommunikation      │        │
│  │                     │    │                     │        │
│  │  - controlTask      │    │  - loop()           │        │
│  │  - 100 Hz           │    │  - micro-ROS Spin   │        │
│  │  - Feedforward      │    │  - Odom Publish     │        │
│  │  - Encoder ISR      │    │  - Heartbeat        │        │
│  │  - Failsafe         │    │                     │        │
│  └──────────┬──────────┘    └──────────┬──────────┘        │
│             │                          │                    │
│             └──────────┬───────────────┘                    │
│                        │                                    │
│              ┌─────────▼─────────┐                         │
│              │   Shared Memory   │                         │
│              │  (Mutex geschützt)│                         │
│              └───────────────────┘                         │
└─────────────────────────────────────────────────────────────┘
                         │
                   USB-CDC (921600 Baud)
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  Raspberry Pi 5 (Docker)                                    │
│                                                             │
│  ┌─────────────────────┐    ┌─────────────────────┐        │
│  │  amr_agent          │    │  amr_dev            │        │
│  │  micro-ROS Agent    │    │  ROS 2 Humble       │        │
│  │  /dev/ttyACM0       │    │  Workspace          │        │
│  └─────────────────────┘    └─────────────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Abstraction Layer (HAL)

Die Pin-Belegung wurde für die **komplette Hardware-Nutzung** refaktoriert. Alle Pins sind initialisiert, um Floating-States zu vermeiden.

| Ressource | Pin | PWM-Kanal | Core | Funktion | Status |
|-----------|-----|-----------|------|----------|--------|
| **Motor L-A** | D0 | CH 1 | Core 0 | PWM Vorwärts | ✅ Aktiv |
| **Motor L-B** | D1 | CH 0 | Core 0 | PWM Rückwärts | ✅ Aktiv |
| **Motor R-A** | D2 | CH 3 | Core 0 | PWM Vorwärts | ✅ Aktiv |
| **Motor R-B** | D3 | CH 2 | Core 0 | PWM Rückwärts | ✅ Aktiv |
| **Encoder L** | D6 | – | Core 0 | ISR (Rising Edge) | ✅ Aktiv |
| **Encoder R** | D7 | – | Core 0 | ISR (Rising Edge) | ✅ Aktiv |
| **Safety/LED** | D10 | – | Core 0 | MOSFET (Not-Aus) | ✅ Aktiv |
| **I2C SDA** | D4 | – | Core 1 | IMU (MPU6050) | ⏳ Reserviert |
| **I2C SCL** | D5 | – | Core 1 | IMU (MPU6050) | ⏳ Reserviert |
| **Servo Pan** | D8 | – | Core 1 | Kamera Pan | ⏳ Reserviert |
| **Servo Tilt** | D9 | – | Core 1 | Kamera Tilt | ⏳ Reserviert |

**Wichtig:** Die PWM-Kanäle wurden getauscht (A↔B), um die korrekte Fahrtrichtung zu erreichen:

```cpp
// config.h
#define PWM_CH_LEFT_A  1  // war 0
#define PWM_CH_LEFT_B  0  // war 1
#define PWM_CH_RIGHT_A 3  // war 2
#define PWM_CH_RIGHT_B 2  // war 3
```

---

## 3. Firmware-Logik (`main.cpp`)

Die Firmware basiert auf zwei parallelen Tasks.

### 3.1 Task: `controlTask` (Core 0)

Dies ist das „Rückenmark" des Roboters.

| Eigenschaft | Wert |
|-------------|------|
| Frequenz | 100 Hz (fixiert durch `vTaskDelayUntil`) |
| Priorität | `configMAX_PRIORITIES - 1` |
| Stack | 4096 Bytes |

**Logik:**

1. **Mutex Lock:** Zielwerte aus Shared Memory lesen
2. **Atomic Read:** Encoder-Werte auslesen (Interrupts gesperrt)
3. **Odometrie:** Position berechnen (x, y, θ)
4. **Richtungs-Heuristik:** Encoder-Vorzeichen aus Soll-Geschwindigkeit ableiten
5. **Feedforward + PID:** Stellgrößen berechnen
6. **Safety:** Prüfen ob `last_cmd_time > 2000ms` → Motoren aus
7. **Mutex Lock:** Odometrie in Shared Memory schreiben

### 3.2 Steuerungslogik (Feedforward)

```cpp
// Inverse Kinematik: Twist → Radgeschwindigkeiten
float set_v_l = target_v - (target_w * WHEEL_BASE / 2.0f);
float set_v_r = target_v + (target_w * WHEEL_BASE / 2.0f);

// Feedforward + PID (PID aktuell deaktiviert: Kp=Ki=Kd=0)
float feedforward_gain = 2.0f;
float pwm_l = feedforward_gain * set_v_l + pid_left.compute(set_v_l, v_enc_l, dt);
float pwm_r = feedforward_gain * set_v_r + pid_right.compute(set_v_r, v_enc_r, dt);

// Begrenzen auf PWM-Bereich
pwm_l = constrain(pwm_l, -1.0f, 1.0f);
pwm_r = constrain(pwm_r, -1.0f, 1.0f);

// Hardware ansteuern
hal_motor_write(pwm_l, pwm_r);
```

**Warum Feedforward statt PID?**

Die Encoder liefern nur Single-Channel-Signale (A-only), daher keine echte Richtungserkennung. Die Richtung wird aus der Soll-Geschwindigkeit abgeleitet. Bei aktivem PID führte dies zu Eskalation (Motor dreht → Encoder zählt → PID interpretiert falsch → mehr Gas). Feedforward umgeht dieses Problem.

### 3.3 Task: `loop` (Core 1)

Dies ist das „Sprachzentrum".

| Eigenschaft | Wert |
|-------------|------|
| Frequenz | Best Effort |
| Odom Publish | 20 Hz (alle 50ms) |
| Heartbeat | 1 Hz |

**Logik:**

1. `rclc_executor_spin_some`: Prüft auf neue `/cmd_vel` Pakete
2. **Odom Publish:** Alle 50ms Position aus Shared Memory lesen und senden
3. **Heartbeat:** Alle 1000ms Counter incrementieren und senden
4. **Sync:** Zugriff auf Shared Memory nur via `xSemaphoreTake`

---

## 4. Schnittstellen & Datenfluss

### 4.1 ROS 2 Topics

| Topic | Typ | Richtung | QoS | Frequenz | Beschreibung |
|-------|-----|----------|-----|----------|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | Reliable | – | Geschwindigkeitsbefehle |
| `/odom_raw` | `geometry_msgs/Pose2D` | Pub | Best Effort | 20 Hz | Odometrie (x, y, theta) |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | Best Effort | 1 Hz | Lebenszeichen |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | Reliable | – | LED/MOSFET Steuerung |

### 4.2 Nachrichtenformate

**cmd_vel (Input):**

```yaml
linear:
  x: 0.15    # [m/s] Vorwärts (+) / Rückwärts (-)
angular:
  z: 0.5     # [rad/s] Links (+) / Rechts (-)
```

**odom_raw (Output):**

```yaml
x: 0.899     # [m] Position X
y: -0.329    # [m] Position Y
theta: 6.09  # [rad] Orientierung
```

### 4.3 Shared Memory Struktur

```cpp
struct SharedData {
    // Input (Core 1 → Core 0)
    float target_lin_x;          // Soll-Linear-Geschwindigkeit [m/s]
    float target_ang_z;          // Soll-Winkel-Geschwindigkeit [rad/s]
    bool led_cmd_active;         // LED-Status
    unsigned long last_cmd_time; // Zeitstempel für Failsafe

    // Output (Core 0 → Core 1)
    float odom_x;                // Position X [m]
    float odom_y;                // Position Y [m]
    float odom_theta;            // Orientierung [rad]
};
```

---

## 5. Konfiguration (`config.h`)

### 5.1 Timing

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `LOOP_RATE_HZ` | 100 | Control-Loop Frequenz |
| `ODOM_PUBLISH_HZ` | 20 | Odometrie Publish Rate |
| `FAILSAFE_TIMEOUT_MS` | 2000 | Heartbeat-Timeout |

### 5.2 Kinematik

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `WHEEL_DIAMETER` | 0.065 m | Raddurchmesser |
| `WHEEL_BASE` | 0.178 m | Spurbreite |
| `TICKS_PER_REV_LEFT` | 374.3 | Encoder-Ticks pro Umdrehung |
| `TICKS_PER_REV_RIGHT` | 373.6 | Encoder-Ticks pro Umdrehung |

### 5.3 Regelung

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `PID_KP` | 0.0 | **Deaktiviert** |
| `PID_KI` | 0.0 | **Deaktiviert** |
| `PID_KD` | 0.0 | **Deaktiviert** |
| `feedforward_gain` | 2.0 | Direkte Ansteuerung |

### 5.4 PWM

| Parameter | Wert | Beschreibung |
|-----------|------|--------------|
| `MOTOR_PWM_FREQ` | 20000 | 20 kHz (unhörbar) |
| `MOTOR_PWM_BITS` | 8 | 0-255 Auflösung |
| `PWM_DEADZONE` | 35 | Mindest-PWM |

---

## 6. Deployment

### 6.1 Firmware Update (Mac)

```bash
cd ~/daten/start/IoT/AMR/amr-platform/firmware
pio run -e seeed_xiao_esp32s3 -t upload
```

Nach dem Upload blinkt die LED, bis der Agent verbunden ist.

### 6.2 Docker starten (Pi)

```bash
cd ~/amr-platform/docker
docker compose up -d
sleep 5
docker compose logs microros_agent --tail 5
```

**Erwartung:** `running... | fd: 3`

### 6.3 Nach ESP32 Reboot

```bash
docker compose restart microros_agent
sleep 5
```

---

## 7. Testing

### 7.1 Verbindung prüfen

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
```

### 7.2 Heartbeat prüfen

```bash
ros2 topic echo /esp32/heartbeat
```

**Erwartung:** Counter incrementiert ~1×/s

### 7.3 Odometrie prüfen

```bash
ros2 topic echo /odom_raw --once
```

**Erwartung:** x, y, theta Werte

### 7.4 Motor-Test (⚠️ Räder aufbocken!)

```bash
# Vorwärts
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10

# Ctrl+C → Failsafe stoppt nach 2s
```

### 7.5 Alle Richtungen testen

| Befehl | Erwartung |
|--------|-----------|
| `linear.x: 0.15` | Vorwärts |
| `linear.x: -0.15` | Rückwärts |
| `angular.z: 0.5` | Drehen links |
| `angular.z: -0.5` | Drehen rechts |

---

## 8. Troubleshooting

| Problem | Ursache | Lösung |
|---------|---------|--------|
| **Motor reagiert nicht** | Feedforward zu niedrig | `feedforward_gain` erhöhen |
| **PID eskaliert** | Encoder-Polarität invertiert | PID deaktivieren (Kp=0) |
| **Räder drehen falsch** | PWM-Kanäle | A↔B tauschen |
| **Failsafe greift zu früh** | Timeout zu kurz | `FAILSAFE_TIMEOUT_MS` erhöhen |
| **Topics fehlen** | Agent nicht verbunden | `docker compose restart microros_agent` |
| **Keine Odom-Daten** | QoS Mismatch | Best Effort QoS nutzen |

---

## 9. Bekannte Einschränkungen

1. **Open-Loop-Steuerung:** PID deaktiviert, keine Geschwindigkeitsregelung
2. **Encoder A-only:** Richtung wird aus Soll-Geschwindigkeit abgeleitet
3. **Odom-Rate:** Effektiv ~3-6 Hz durch Serial-Transport
4. **Keine TF:** `odom` → `base_link` Transform muss extern erfolgen

---

## 10. Nächste Entwicklungsschritte

| Phase | Aufgabe | Status |
|-------|---------|--------|
| Phase 3 | RPLidar A1 Integration | 🔜 |
| Phase 4 | EKF Sensor Fusion + TF | ⏳ |
| Phase 4 | `odom_converter.py` Bridge Node | ⏳ |
| Phase 5 | SLAM (slam_toolbox) | ⏳ |
| Phase 6 | Nav2 Navigation | ⏳ |

---

## 11. Changelog

### v3.2.0 (20.12.2025) – Phase 1 Abschluss

- **Motor-Richtung:** PWM-Kanäle getauscht (A↔B)
- **Steuerung:** Feedforward (Gain=2.0) statt PID
- **PID:** Deaktiviert wegen Encoder-Polarität
- **Failsafe:** Timeout auf 2000ms erhöht
- **Tests:** Alle Richtungen validiert

### v3.1.0 (20.12.2025)

- **Baudrate:** 921600
- **PID:** Aktiviert (Kp=1.0)
- **Problem:** PID-Eskalation

### v3.0.0 (14.12.2025)

- **Architektur:** Dual-Core
- **RTOS:** FreeRTOS Tasks + Mutex

---

*Diese Dokumentation dient als "Single Source of Truth" für die Firmware-Entwicklung.*
