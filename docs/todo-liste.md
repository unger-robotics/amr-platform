# ToDo-Liste AMR-Projekt

> **Stand:** 2025-12-13 | **Aktuelle Phase:** 3.3 (Odometrie)

---

## 📊 Phasen-Übersicht (Revidiert)

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 0 | Fundament (OS, Docker, Hailo) | ✅ Abgeschlossen |
| Phase 1 | Motor-Test + Teleop (Serial-Bridge) | ✅ Abgeschlossen |
| Phase 2 | Encoder + Odometrie + PID (Serial-Bridge) | ✅ Abgeschlossen |
| Phase 3 | **micro-ROS Integration** | ◄── **AKTUELL** |
| Phase 4 | LiDAR + SLAM | ⬜ |
| Phase 5 | Navigation (Nav2) | ⬜ |
| Phase 6 | Kamera + AI | ⬜ |
| Phase 7 | Integration & Härtung | ⬜ |

---

## ✅ Phase 1 & 2: Abgeschlossen (Serial-Bridge)

### Erreichte Ziele (als Backup vorhanden)

- [x] ESP32 Serial-Bridge Firmware v0.5.0-pid
- [x] Differential Drive mit PID-Regelung
- [x] Encoder-Kalibrierung (374.3 / 373.6 Ticks/Rev)
- [x] Odometrie (x, y, theta)
- [x] ROS 2 Serial Bridge Node
- [x] Bodentest: 1.6% Distanzfehler, 0.5cm Drift

**Hinweis:** Serial-Bridge bleibt als Fallback in `firmware_serial/`

---

## 🎯 Phase 3: micro-ROS Integration (AKTUELL)

### Architektur-Entscheidung (2025-12-13)

**Alt:** Serial-Bridge (Python Parser, Custom Protokoll)
**Neu:** micro-ROS (Native ROS 2 Topics, DDS-Standard)

| Aspekt | Serial-Bridge | micro-ROS |
|--------|---------------|-----------|
| Protokoll | Custom Text | DDS/XRCE |
| ROS 2 Integration | Bridge-Node | Native |
| Zukunftssicherheit | Begrenzt | ✅ Standard |
| Komplexität | Einfacher | Höher |

### 3.1 Agent als systemd Service ✅

- [x] Service-Datei erstellen (`/etc/systemd/system/microros-agent.service`)
- [x] Automatischer Start bei Boot
- [x] Restart bei Absturz
- [x] Status-Monitoring

**Validiert:** 2025-12-13

### 3.2 Motor-Control (`/cmd_vel`) ✅

- [x] `geometry_msgs/Twist` Subscriber implementieren
- [x] Differential Drive Kinematik (v, ω → v_left, v_right)
- [x] PWM-Ausgabe an Cytron MDD3A
- [x] Deadzone-Kompensation
- [x] Failsafe (Timeout → Motoren stopp)
- [x] Teleop Tastatursteuerung getestet

**Topics (aktiv):**

| Topic | Typ | Richtung | Status |
|-------|-----|----------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Sub | ✅ |
| `/esp32/heartbeat` | `std_msgs/Int32` | Pub | ✅ |
| `/esp32/led_cmd` | `std_msgs/Bool` | Sub | ✅ |

**Validiert:** 2025-12-13

### 3.3 Odometrie (`/odom`) ◄── AKTUELL

- [ ] Encoder-ISR implementieren (D6, D7)
- [ ] Tick-Zählung (Interrupt-basiert)
- [ ] Odometrie-Berechnung (Δx, Δy, Δθ)
- [ ] `nav_msgs/Odometry` Publisher
- [ ] TF-Broadcast: `odom` → `base_link`

**Geplante Topics:**

| Topic | Typ | Frequenz | Beschreibung |
|-------|-----|----------|--------------|
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Position & Orientierung |
| `/tf` | `tf2_msgs/TFMessage` | 50 Hz | Transform odom→base_link |

### 3.4 IMU Integration (Optional) ⬜

- [ ] MPU6050 über I2C ansprechen (D4/D5)
- [ ] `sensor_msgs/Imu` Publisher
- [ ] Orientierung (Quaternion)
- [ ] TF: `base_link` → `imu_link`

---

## 📋 Phase 3 Validierung

| Test | Kriterium | Status |
|------|-----------|--------|
| Agent Service | Startet automatisch nach Reboot | ✅ |
| cmd_vel → Motor | Teleop funktioniert | ✅ |
| Failsafe | Motoren stoppen nach 500ms | ✅ |
| Odometrie | 1m Test < 5% Fehler | ⬜ |
| TF Tree | odom → base_link korrekt | ⬜ |

---

## 🔜 Nächste Phasen (Vorschau)

### Phase 4: LiDAR + SLAM

- [ ] RPLIDAR A1 in Docker einbinden
- [ ] slam_toolbox konfigurieren
- [ ] Testraum kartieren
- [ ] Karte speichern (PGM + YAML)

### Phase 5: Navigation (Nav2)

- [ ] Nav2 Stack konfigurieren
- [ ] AMCL Lokalisierung
- [ ] Autonome Punkt-zu-Punkt Navigation

### Phase 6: Kamera + AI

- [ ] IMX296 Global Shutter
- [ ] YOLOv8 auf Hailo-8L
- [ ] Personen-Erkennung → Stopp

### Phase 7: Integration

- [ ] Sensor Fusion (EKF)
- [ ] Demo vorbereiten

---

## 📅 Revidierter Zeitplan

```
Woche:  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18
        ════════════════════════════════════════════════════
Phase 0 ████                                                 Fundament     ✅
Phase 1       ████                                           Motor-Test    ✅
Phase 2             ████                                     Odometrie     ✅
Phase 3                   ████                               micro-ROS     ◄── AKTUELL
Phase 4                         ████                         SLAM
Phase 5                               ██████                 Navigation
Phase 6                                       ████           Kamera/AI
Phase 7                                             ████     Integration
        ════════════════════════════════════════════════════
```

---

## 🔧 Software-Versionen

| Komponente | Version | Ort |
|------------|---------|-----|
| micro-ROS Firmware | **v2.0.0** | `esp32_microros_test/` |
| micro-ROS Agent | Humble (Docker) | systemd Service |
| Serial-Bridge (Backup) | v0.5.0-pid | `firmware_serial/` |

---

## 📁 Projekt-Struktur (aktualisiert)

```
amr-platform/
├── esp32_microros_test/     # ◄── AKTIV (micro-ROS v2.0.0)
│   ├── include/config.h
│   ├── src/main.cpp
│   ├── platformio.ini
│   └── README.md
├── firmware_serial/          # Backup (Serial-Bridge)
├── firmware_test/            # Hardware-Tests
├── docker/
│   └── docker-compose.yml   # serial_bridge entfernt
├── scripts/
│   ├── setup_microros_service.sh
│   └── microros-agent.service
├── ros2_ws/
└── docs/
```

---

## ✅ Checkliste: Phase 3 abgeschlossen wenn

- [x] Agent startet automatisch bei Boot
- [x] `/cmd_vel` steuert Motoren
- [x] Teleop funktioniert
- [x] Failsafe getestet
- [ ] `/odom` publiziert Position
- [ ] TF-Tree ist korrekt
- [ ] Code committet und dokumentiert
- [ ] README.md aktualisiert

---

*Aktualisiert: 2025-12-13 | Phase 3.1 + 3.2 abgeschlossen*
