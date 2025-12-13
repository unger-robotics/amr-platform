# Systemdokumentation: AMR Low-Level Controller

**Version:** 1.4.0 | **Datum:** 13.12.2025 | **Status:** ✅ Validiert (Bidirektionale Kommunikation)

## 1. Architektur-Übersicht

Das System folgt einer asymmetrischen Client-Server-Architektur zur Integration von Echtzeit-Hardware in ein ROS 2 Humble Netzwerk.

* **Node Name:** `esp32_node`
* **Hardware:** Seeed Studio XIAO ESP32-S3
* **Transport-Layer:** Serial (USB CDC) über `/dev/ttyACM0`
* **Middleware:** Micro-XRCE-DDS (via micro-ROS Agent)

**Kommunikationsfluss:**

```
Sensoren/Aktoren ↔ ESP32 Firmware ↔ [USB Serial] ↔ micro-ROS Agent (Docker) ↔ [DDS/UDP] ↔ ROS 2 Netzwerk
```

---

## 2. Hardware Abstraction Layer (HAL)

Die Pin-Belegung ist in `config.h` definiert. Änderungen an der Verkabelung müssen hier reflektiert werden.

### 2.1 Pin-Mapping

| Funktion | Pin (ESP32) | Hardware-Ziel | Beschreibung |
|----------|-------------|---------------|--------------|
| **Debug / Status** | GPIO 21 | Onboard LED (Gelb) | Statusanzeige (Blinkt=Suche, An=Verbunden) |
| **Aktorik 1** | D10 (GPIO) | IRLZ24N MOSFET | Schaltet LED-Streifen / High-Power Last |
| **Motor L** | D0 / D1 | Cytron MDD3A | PWM Signale Motor Links (A/B) |
| **Motor R** | D2 / D3 | Cytron MDD3A | PWM Signale Motor Rechts (A/B) |
| **Encoder L** | D6 | JGA25-370 Hall | Interrupt-Input Links |
| **Encoder R** | D7 | JGA25-370 Hall | Interrupt-Input Rechts |

### 2.2 Serielle Schnittstelle

* **Baudrate:** `115200` (Sync mit Agent zwingend erforderlich)
* **Protokoll:** Custom Serial Transport (micro-ROS)
* **Device:** `/dev/ttyACM0` (Natives USB des S3)

---

## 3. ROS 2 Schnittstellendefinition (API)

Der `esp32_node` stellt folgende Topics im ROS 2 Graphen zur Verfügung.

### 3.1 Publisher (Daten vom Rover)

| Topic | Typ | Frequenz | QoS | Beschreibung |
|-------|-----|----------|-----|--------------|
| `/esp32/heartbeat` | `std_msgs/Int32` | 1 Hz | Best Effort | Zähler (Watchdog). Beweis, dass der Loop läuft. |

### 3.2 Subscriber (Befehle an den Rover)

| Topic | Typ | Callback | Beschreibung |
|-------|-----|----------|--------------|
| `/esp32/led_cmd` | `std_msgs/Bool` | `led_callback` | Steuert MOSFET an D10. `true` = AN, `false` = AUS. |

---

## 4. Firmware-Logik (`main.cpp`)

### 4.1 Lebenszyklus & Verbindungsaufbau

Die Firmware implementiert eine robuste Wiederverbindungsstrategie:

1. **Boot:** Initialisierung der Hardware-Pins (Safety: MOSFETs auf LOW).
2. **USB-CDC Init:** `Serial.begin(115200)` + 2s Delay für USB-Stabilisierung.
3. **Ping-Loop:** Der ESP32 sendet `rmw_uros_ping_agent`.
   * *Feedback:* Onboard LED blinkt schnell (10 Hz).
   * *Blockierend:* Der Code läuft erst weiter, wenn der Agent antwortet.
4. **Session Init:** Sobald Agent gefunden → LED leuchtet dauerhaft.
5. **Entity Creation:** Erstellung von Node, Publisher, Subscriber und Executor.

### 4.2 Executor-Modell

Es wird der `rclc_executor` verwendet, um deterministisches Verhalten zu garantieren.

* **Strategie:** `spin_some` mit 10ms Timeout im `loop()`.
* **Konsequenz:** Der ESP32 kann maximal alle 10ms auf neue ROS-Nachrichten reagieren (ausreichend für LED, für Motoren später < 10ms empfohlen).

### 4.3 QoS-Konfiguration

| Entity | QoS Profil | Grund |
|--------|-----------|-------|
| Publisher (heartbeat) | Best Effort | Weniger Speicher, Paketverlust akzeptabel |
| Subscriber (led_cmd) | Default (Reliable) | Befehle sollen ankommen |

---

## 5. Konfiguration & Physik (`config.h`)

Zentrale Parameter für Kinematik und Regelung.

### 5.1 Fahrzeug-Physik (Differential Drive)

* **Raddurchmesser:** 0.065 m (65mm)
* **Spurbreite:** 0.178 m (178mm)
* **Encoder-Auflösung:** ~374 Ticks/Umdrehung (Kalibriert am 12.12.2025)

### 5.2 PID-Regler (Velocity)

Werte basierend auf Tuning-Session (Boden-Test):

* `PID_KP = 13.0` (Aggressive Annäherung an Sollwert)
* `PID_KI = 5.0` (Starke Korrektur gegen stationäre Fehler/Reibung)
* `PID_KD = 0.01` (Minimale Dämpfung nötig)

---

## 6. Betriebsanleitung (Operator Manual)

### Schritt 1: Start des Agenten (Raspberry Pi)

Der Docker-Container muss privilegiert gestartet werden, um auf USB zuzugreifen.

```bash
docker run -it --rm --net=host --privileged \
    -v /dev:/dev \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyACM0 -b 115200
```

### Schritt 2: Handshake (ESP32)

1. Agent läuft (zeigt `running... | fd: 3`).
2. ESP32 anschließen oder **RESET-Button** drücken.
3. LED wechselt von Blinken auf Dauerlicht.
4. Agent-Log zeigt Entity-Erstellung (optional, nicht immer sichtbar).

### Schritt 3: Test-Befehle (ROS 2 Host)

**Topics auflisten:**

```bash
docker run -it --rm --net=host ros:humble ros2 topic list
```

**Heartbeat anzeigen:**

```bash
docker run -it --rm --net=host ros:humble ros2 topic echo /esp32/heartbeat
```

**Licht AN:**

```bash
docker run -it --rm --net=host ros:humble ros2 topic pub --once /esp32/led_cmd std_msgs/msg/Bool "{data: true}"
```

**Licht AUS:**

```bash
docker run -it --rm --net=host ros:humble ros2 topic pub --once /esp32/led_cmd std_msgs/msg/Bool "{data: false}"
```

---

## 7. Bekannte Warnungen

| Warnung | Ursache | Auswirkung |
|---------|---------|------------|
| `sequence size exceeds remaining buffer` | Artefakt der Serialisierung bei Best Effort QoS über Serial | **Funktional unkritisch** |

---

## 8. Projektstruktur

```
esp32_microros_test/
├── include/
│   └── config.h          # Hardware-Konfiguration, PID-Parameter
├── src/
│   └── main.cpp          # Firmware mit micro-ROS Integration
├── platformio.ini        # Build-Konfiguration (Humble, Serial Transport)
├── extra_script.py       # micro-ROS Build-Hook
└── README.md             # Diese Dokumentation
```

---

## 9. Entwicklungsumgebung

### Build & Flash (Mac)

```bash
cd esp32_microros_test
pio run -t upload
```

### Wichtig: Distribution Sync

| Komponente | Distribution |
|------------|--------------|
| ESP32 Firmware | **Humble** (`board_microros_distro = humble`) |
| Docker Agent | **Humble** (`microros/micro-ros-agent:humble`) |

Beide müssen identisch sein!

### Cache-Probleme beheben

Bei Wechsel der Distribution:

```bash
sudo rm -rf .pio/libdeps/*/micro_ros_platformio/libmicroros
pio run
```

---

## 10. Nächste Entwicklungsschritte

| Priorität | Feature | Topic | Message Type |
|-----------|---------|-------|--------------|
| 1 | Motor-Control | `/cmd_vel` | `geometry_msgs/Twist` |
| 2 | Odometrie | `/odom` | `nav_msgs/Odometry` |
| 3 | IMU-Integration | `/imu` | `sensor_msgs/Imu` |
| 4 | Agent als Service | systemd | - |

---

## Changelog

### v1.4.0 (13.12.2025)

- ✅ micro-ROS Kommunikation validiert
* ✅ Bidirektionale Kommunikation (Heartbeat + LED Control)
* ✅ Dokumentation aktualisiert

### v1.3.0 (12.12.2025)

- PID-Parameter für Geschwindigkeitsregelung
* Encoder-Kalibrierung abgeschlossen
