# AMR Implementierungsplan

## Vom Schaltplan zur autonomen Navigation

> **Version:** 1.3 | **Stand:** 2025-12-12 | **Firmware:** v0.5.0-pid

---

## Das Grundprinzip: Vertikale Scheiben statt horizontaler Schichten

Ein häufiger Fehler bei Robotik-Projekten: Man baut zuerst die gesamte Hardware auf, dann die gesamte Firmware, dann die gesamten Treiber – und am Ende, beim ersten Integrationstest, funktioniert nichts. Die Fehlersuche wird zum Albtraum, weil alles gleichzeitig neu ist.

**Unser Ansatz:** Wir schneiden das System in *vertikale Scheiben*. Jede Phase liefert ein lauffähiges Teilsystem, das wir testen können, bevor die nächste Komplexitätsstufe hinzukommt.

```
Klassisch (riskant):          Unser Weg (inkrementell):

┌──────────────────┐          Phase 1: ──────────────────► ✅
│    Navigation    │                   Motor + Failsafe
├──────────────────┤
│   Wahrnehmung    │          Phase 2: ──────────────────► ✅
├──────────────────┤                   + Encoder + Odom + PID
│    Firmware      │
├──────────────────┤          Phase 3: ──────────────────► ◄── AKTUELL
│    Hardware      │                   + LiDAR + SLAM
└──────────────────┘
       ↓                      Phase 4: ──────────────────►
  Big Bang Test                        + Navigation
  (Chaos)
                              Phase 5: ──────────────────►
                                       + Kamera + AI
```

---

## Phase 0: Fundament (Woche 1–2) ✅

**Ziel:** Eine saubere Entwicklungsumgebung, die uns später keine Steine in den Weg legt.

**Status:** ✅ Abgeschlossen

- Raspberry Pi OS Lite (64-bit, Bookworm)
- Docker und Docker Compose
- Hailo-8L Treiber (HailoRT 4.23.0)
- Git-Workflow Mac ↔ GitHub ↔ Pi

---

## Phase 1: Der erste Lebenshauch (Woche 3–4) ✅

**Ziel:** Ein Rad dreht sich auf Befehl. Das klingt trivial – ist aber der Beweis, dass die gesamte Kette von ROS 2 bis zum Motor funktioniert.

**Status:** ✅ Abgeschlossen (2025-12-12)

### 1.1 Erreichte Meilensteine

| Komponente | Status |
|------------|--------|
| ESP32 Serial-Bridge Firmware | ✅ v0.3.0-serial |
| Differential Drive Kinematik | ✅ |
| Deadzone-Kompensation | ✅ |
| Failsafe (500ms Timeout) | ✅ |
| ROS 2 Serial Bridge Node | ✅ |
| Docker Integration | ✅ |
| Teleop Tastatursteuerung | ✅ |

### 1.2 Architektur-Entscheidung

**Problem:** micro-ROS Build scheitert an Python 3.13 (Raspberry Pi OS Bookworm)

**Lösung:** Serial-Bridge als Workaround

### 1.3 Cytron MDD3A – Dual-PWM Steuerung

> ⚠️ **Kritisch:** Der MDD3A verwendet **kein** DIR-Pin, sondern zwei PWM-Signale pro Motor!

| M1A (PWM) | M1B (PWM) | Ergebnis |
|-----------|-----------|----------|
| 200 | 0 | Vorwärts |
| 0 | 200 | Rückwärts |
| 0 | 0 | Coast (Auslaufen) |
| 200 | 200 | Active Brake |

**Meilenstein Phase 1:** ✅ Teleop funktioniert, Failsafe aktiv.

---

## Phase 2: Bewegung mit Feedback (Woche 5–6) ✅

**Ziel:** Der Roboter weiß, wo er ist (Odometrie) und fährt präzise geradeaus (PID-Regelung).

**Status:** ✅ Abgeschlossen (2025-12-12)

### 2.1 Encoder-Kalibrierung

**Methode:** 10-Umdrehungen-Test mit `calibration_encoder.cpp`

| Rad | Ticks (10 Umdrehungen) | Ticks/Rev | Theoretisch |
|-----|------------------------|-----------|-------------|
| Links | 3743 | **374.3** | 390.5 |
| Rechts | 3736 | **373.6** | 390.5 |

Die kalibrierten Werte weichen 4,2 % vom theoretischen Wert ab (Getriebespiel, Toleranzen).

### 2.2 Odometrie-Berechnung

**Differentialkinematik:**

```
d_left  = (delta_ticks_left / TICKS_PER_REV) × WHEEL_CIRCUMFERENCE
d_right = (delta_ticks_right / TICKS_PER_REV) × WHEEL_CIRCUMFERENCE

d_center = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / WHEEL_BASE

x += d_center × cos(theta + d_theta/2)
y += d_center × sin(theta + d_theta/2)
theta += d_theta
```

### 2.3 PID-Geschwindigkeitsregelung

**Problem (Open-Loop):**

- Distanzfehler: 16 % (1.158 m statt 1.0 m)
- Drift: 14 cm nach links (rechtes Rad 3,7 % schneller)

**Lösung:** Closed-Loop PID-Regelung pro Rad

```
                    ┌─────────────────────────────────────────────────────┐
                    │              Pro Rad (links/rechts)                 │
                    │                                                     │
  Soll-v ──────────►│  ┌───────┐      ┌───────┐      ┌───────┐          │
  (cmd_vel)         │  │  PID  │──────│  PWM  │──────│ Motor │──────┬───│───► Rad
                    │  │Regler │      │Treiber│      │       │      │   │
                    │  └───────┘      └───────┘      └───────┘      │   │
                    │       ▲                                       │   │
                    │       │         ┌───────────┐                 │   │
                    │       └─────────│  Encoder  │◄────────────────┘   │
                    │         Ist-v   │  → v_ist  │                     │
                    │                 └───────────┘                     │
                    └─────────────────────────────────────────────────────┘
```

### 2.4 PID-Tuning Ergebnis

**Tuning-Methode:** Manuell (inkrementell) am 2025-12-12

| Parameter | Startwert | Endwert | Funktion |
|-----------|-----------|---------|----------|
| **Kp** | 2.0 | **13.0** | Hauptkorrektur – erhöht bis Soll-v erreicht |
| **Ki** | 0.5 | **5.0** | Stationärer Fehler – eliminiert Drift |
| **Kd** | 0.01 | **0.01** | Dämpfung – kein Überschwingen beobachtet |

### 2.5 Validierung (Bodentest 1m @ 0.2 m/s)

| Metrik | Open-Loop | Mit PID | Verbesserung |
|--------|-----------|---------|--------------|
| Distanz x | 1.158 m | **0.984 m** | Fehler: 16% → **1.6%** |
| Drift y | 14 cm | **0.5 cm** | **28× besser** |
| Encoder L/R | 2381/2470 | **1802/1802** | Synchron! |

### 2.6 Serial-Protokoll (erweitert)

```
ESP32 → Host: ODOM:<left_ticks>,<right_ticks>,<x>,<y>,<theta>\n
Host → ESP32: PID:<Kp>,<Ki>,<Kd>\n   (Live-Tuning)
Host → ESP32: DEBUG:ON/OFF\n         (Velocity-Nachrichten)
```

### 2.7 ROS 2 Integration

- `/odom` (Typ: `nav_msgs/Odometry`) – Position und Orientierung
- TF-Broadcast: `odom` → `base_link`

**Meilenstein Phase 2:** ✅ Odometrie <2% Fehler, Drift <1cm, PID-Regelung aktiv.

---

## Phase 3: Sehen lernen – LiDAR & SLAM (Woche 7–9) ◄── AKTUELL

**Ziel:** Der Roboter baut eine Karte seiner Umgebung.

### 3.1 LiDAR-Treiber

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### 3.2 SLAM-Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=slam_params.yaml
```

**Meilenstein Phase 3:** Eine speicherbare Karte des Testraums existiert.

---

## Phase 4: Autonome Navigation (Woche 10–12)

**Ziel:** Wir setzen ein Ziel auf der Karte, der Roboter fährt autonom hin.

### 4.1 Nav2 Stack

| Komponente | Funktion |
|------------|----------|
| **AMCL** | Lokalisierung auf bekannter Karte |
| **Planner Server** | Globaler Pfad (A* / Dijkstra) |
| **Controller Server** | Lokale Hindernisvermeidung |
| **Costmap** | Hinderniskarte aus Sensordaten |
| **BT Navigator** | Verhaltenssteuerung |

**Meilenstein Phase 4:** Roboter navigiert autonom, weicht Hindernissen aus.

---

## Phase 5: Wahrnehmungserweiterung – Kamera & AI (Woche 13–15)

**Ziel:** Der Roboter erkennt Objekte und kann darauf reagieren.

- IMX296 Global Shutter Kamera
- YOLOv8 auf Hailo-8L (31 FPS validiert)
- Personen-Erkennung → Stopp-Verhalten

**Meilenstein Phase 5:** Roboter stoppt, wenn eine Person erkannt wird.

---

## Phase 6: Integration & Härtung (Woche 16–18)

**Ziel:** Robustes System für Demo und Dokumentation.

- Sensor Fusion (EKF)
- Systemstart automatisieren
- Demo: Karte, Waypoints, Video

**Meilenstein Phase 6:** Robustes System, startet automatisch, Demo-fähig.

---

## Risikomatrix

| Risiko | Wahrscheinlichkeit | Impact | Mitigation |
|--------|-------------------|--------|------------|
| libcamera-Inkompatibilität | Niedrig | Hoch | ✅ Raspberry Pi OS |
| Odometrie-Drift | ~~Hoch~~ | ~~Mittel~~ | ✅ **PID-Regelung** |
| Hailo-Treiber instabil | Mittel | Mittel | Navigation funktioniert auch ohne AI |
| Nav2-Tuning aufwändig | Hoch | Mittel | Viel Zeit einplanen |
| micro-ROS inkompatibel | ~~Hoch~~ | ~~Hoch~~ | ✅ **Serial-Bridge** |
| MDD3A-Ansteuerung | ~~Hoch~~ | ~~Hoch~~ | ✅ **Dual-PWM** |

---

## Zeitplan (Übersicht)

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

## Checkliste pro Phase

Jede Phase ist erst abgeschlossen, wenn:

- [x] Die definierten Tests bestanden sind
- [x] Der Code committet und dokumentiert ist
- [x] Die Konfigurationsdateien versioniert sind
- [x] Ein kurzes Protokoll die Ergebnisse festhält
- [x] Der nächste Schritt klar ist

**Phase 1:** ✅ Alle Punkte erfüllt
**Phase 2:** ✅ Alle Punkte erfüllt

---

*Dieser Plan folgt dem Prinzip: Jede Woche ein lauffähiges System. Lieber weniger Features, die funktionieren, als viele Features, die zusammen crashen.*

*Aktualisiert: 2025-12-12 | Firmware: v0.5.0-pid | Phase 2 abgeschlossen*
