# ToDo-Liste AMR-Projekt

> **Stand:** 2025-12-12 | **Aktuelle Phase:** 3 (SLAM)

---

## 📊 Phasen-Übersicht

| Phase | Beschreibung | Status |
|-------|--------------|--------|
| Phase 0 | Fundament (OS, Docker, Hailo) | ✅ Abgeschlossen |
| Phase 1 | Motor-Test + Teleop | ✅ Abgeschlossen |
| Phase 2 | Encoder + Odometrie + PID | ✅ **Abgeschlossen** |
| Phase 3 | LiDAR + SLAM | ◄── **AKTUELL** |
| Phase 4 | Navigation | ⬜ |
| Phase 5 | Kamera + AI | ⬜ |
| Phase 6 | Integration | ⬜ |

---

## ✅ Phase 1: Abgeschlossen (2025-12-12)

### Erreichte Ziele

- [x] ESP32 Serial-Bridge Firmware v0.3.0
- [x] Differential Drive Kinematik
- [x] Deadzone-Kompensation
- [x] Failsafe (500ms Timeout)
- [x] ROS 2 Serial Bridge Node
- [x] Docker Integration
- [x] Teleop Tastatursteuerung
- [x] Git-Workflow Mac ↔ GitHub ↔ Pi

### Workaround dokumentiert

- micro-ROS Build scheitert an Python 3.13
- **Lösung:** Serial-Bridge statt micro-ROS Agent

---

## ✅ Phase 2: Abgeschlossen (2025-12-12)

### 2.1 Encoder-Kalibrierung

- [x] Kalibrierungs-Sketch auf ESP32 flashen
- [x] Linkes Rad: 10 Umdrehungen → 3743 Ticks
- [x] Rechtes Rad: 10 Umdrehungen → 3736 Ticks
- [x] `TICKS_PER_REV_LEFT = 374.3f` in config.h
- [x] `TICKS_PER_REV_RIGHT = 373.6f` in config.h

### 2.2 ESP32 Firmware erweitern

- [x] Encoder-ISR implementieren (D6, D7)
- [x] Odometrie-Berechnung (x, y, theta)
- [x] Serial-Protokoll: `ODOM:<l>,<r>,<x>,<y>,<theta>\n`
- [x] `RESET_ODOM` Befehl
- [x] PID-Geschwindigkeitsregelung (Kp=13, Ki=5, Kd=0.01)
- [x] Live-Tuning: `PID:<Kp>,<Ki>,<Kd>` Befehl
- [x] Debug-Modus: `DEBUG:ON/OFF` für VEL-Nachrichten

### 2.3 ROS 2 Bridge erweitern

- [x] Odometrie parsen
- [x] `/odom` Topic publizieren (nav_msgs/Odometry)
- [x] TF-Broadcast: `odom` → `base_link`

### 2.4 Validierung (Bodentest)

| Test | Soll | Ist | Status |
|------|------|-----|--------|
| 1m Geradeaus | x=1.0m | x=0.984m | ✅ 1.6% Fehler |
| Drift | y=0.0m | y=0.005m | ✅ 0.5cm |
| Encoder-Sync | gleich | 1802/1802 | ✅ Perfekt |

### Vergleich Open-Loop vs. PID

| Metrik | Open-Loop | Mit PID | Verbesserung |
|--------|-----------|---------|--------------|
| Distanzfehler | 16% | 1.6% | **10× besser** |
| Drift | 14 cm | 0.5 cm | **28× besser** |

---

## 🎯 Phase 3: SLAM (AKTUELL)

### 3.1 LiDAR Integration

- [ ] RPLIDAR A1 in Docker einbinden
- [ ] `/scan` Topic verifizieren
- [ ] TF: `base_link` → `laser_frame`

### 3.2 SLAM Toolbox

- [ ] slam_toolbox konfigurieren
- [ ] Online Async SLAM starten
- [ ] Testraum kartieren

### 3.3 Validierung

- [ ] Karte speichern (PGM + YAML)
- [ ] Karte in RViz2 visualisieren
- [ ] Lokalisierungsgenauigkeit prüfen

---

## 📋 Nächste Phasen (Vorschau)

### Phase 4: Navigation

- [ ] Nav2 Stack konfigurieren
- [ ] AMCL Lokalisierung
- [ ] Autonome Punkt-zu-Punkt Navigation

### Phase 5: Kamera + AI

- [ ] IMX296 Global Shutter integrieren
- [ ] YOLOv8 auf Hailo-8L
- [ ] Personen-Erkennung → Stopp-Verhalten

### Phase 6: Integration

- [ ] Sensor Fusion (EKF)
- [ ] Systemstart automatisieren
- [ ] Demo vorbereiten

---

## 📚 Dokumentation

| Datei | Inhalt | Status |
|-------|--------|--------|
| `01-Pi-OS-flashen.md` | OS-Installation, SSH, Docker | ✅ |
| `02-hailo-setup.md` | HailoRT 4.23.0, Benchmark | ✅ |
| `03-ros2-docker.md` | Container-Setup, URDF | ✅ |
| `04-esp32-firmware.md` | PlatformIO Firmware | ✅ |
| `08-entwicklerdoku-status.md` | Projektstatus | ✅ Aktualisiert |
| `AMR_Implementierungsplan.md` | Phasenplan | ✅ |
| `Industriestandards-AMR.md` | REP-103, REP-105 | ✅ |

---

## 📅 Zeitplan

```
Woche:  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18
        ════════════════════════════════════════════════════
Phase 0 ████                                                 Fundament     ✅
Phase 1       ████                                           Motor-Test    ✅
Phase 2             ████                                     Odometrie     ✅
Phase 3                   ██████                             SLAM          ◄── AKTUELL
Phase 4                            ██████                    Navigation
Phase 5                                     ██████           Kamera/AI
Phase 6                                              ██████  Integration
        ════════════════════════════════════════════════════
```

---

## ✅ Checkliste pro Phase

Jede Phase ist erst abgeschlossen, wenn:

- [x] Die definierten Tests bestanden sind
- [x] Der Code committet und dokumentiert ist
- [x] Die Konfigurationsdateien versioniert sind
- [x] Ein kurzes Protokoll die Ergebnisse festhält
- [x] Der nächste Schritt klar ist

**Phase 1:** ✅ Alle Punkte erfüllt
**Phase 2:** ✅ Alle Punkte erfüllt

---

## 🔧 Aktuelle Software-Versionen

| Komponente | Version |
|------------|---------|
| ESP32 Firmware | **v0.5.0-pid** |
| Serial Bridge | v0.4.0-odom |
| Docker Stack | perception + serial_bridge |
| Git Repo | ju1-eu/amr-platform |

---

*Aktualisiert: 2025-12-12 | Phase 2 abgeschlossen*
