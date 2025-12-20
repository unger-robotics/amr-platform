---
title: "Phase 5 – SLAM Toolbox: Mapping (/map) + map→odom TF"
status: "active"
updated: "2025-12-19"
depends_on:
  - "Phase 2 (ROS 2 Jazzy via Docker auf Pi 5)"
  - "Phase 2.5 (URDF + TF Frames)"
  - "Phase 3 (RPLidar A1 + /scan + RViz2)"
  - "Phase 4 (robot_localization EKF: /odometry/filtered + odom→base_footprint)"
next:
  - "Phase 6 (Nav2 Navigation auf Map)"
---

# Phase 5: SLAM Toolbox (Mapping)

SLAM Toolbox erzeugt aus **LaserScan** + **Odometrie** eine **Occupancy-Grid-Map** (`/map`) und stellt typischerweise zusätzlich die TF **`map → odom`** bereit. Damit ist es eine gängige SLAM-Option für Nav2.
(Phase 6 setzt voraus: `/map` + `map→odom` sind verfügbar.)

---

## Zielbild & Definition of Done

### Zielbild

- SLAM Toolbox läuft auf dem Pi 5 (ROS 2 Jazzy in Docker).
- `/scan` liefert saubere LaserScans (Phase 3).
- EKF liefert stabiles `/odometry/filtered` und TF `odom → base_footprint` (Phase 4).
- SLAM Toolbox publiziert:
  - `/map` (`nav_msgs/OccupancyGrid`)
  - TF `map → odom` (für Nav2 / RViz2)
- Map wird als `.yaml` + `.pgm` gespeichert (via `map_saver_cli`).

### DoD (prüfbar)

- [ ] `ros2 topic list` enthält `/map` und `/scan`
- [ ] `ros2 run tf2_ros tf2_echo map odom` liefert fortlaufend gültige Transform
- [ ] RViz2: `Map` sichtbar, Scan liegt plausibel in der Karte, Roboterpose bewegt sich konsistent
- [ ] Map-Save erzeugt Dateien: `<name>.yaml` + `<name>.pgm`

---

## 1) Eingangsgrößen & TF-Kontrakt (muss vorher stimmen)

### 1.1 Topics

- Laser: `/scan` (`sensor_msgs/LaserScan`)
  **frame_id**: `base_laser` (aus Phase 2.5)
- Odom: empfohlen als **Standard-Odom** nach Phase 4:
  - `/odometry/filtered` (oder remapped auf `/odom`)

### 1.2 TF-Baum (minimal)

- URDF / robot_state_publisher (statisch):
  `base_footprint → base_link → base_laser`
- EKF (dynamisch):
  `odom → base_footprint`
- SLAM Toolbox (dynamisch):
  `map → odom`

**Regel:** Genau **eine** Quelle pro TF-Kante (kein Doppel-Publishing von `odom→base_*`).

---

## 2) Installation (Jazzy, Docker)

### 2.1 Paket installieren (im ROS-Container)

```bash
sudo apt update
sudo apt install -y ros-jazzy-slam-toolbox
```

### 2.2 Optional: aus Source bauen (nur wenn nötig)

Wenn du Patches brauchst oder Binary nicht passt: in `ros2_ws/src` clonen und `colcon build`.
(Erstmal nicht machen, wenn „zukunftssicher + stabil“ dein Primärziel ist.)

---

## 3) Start: Online Mapping (async)

Nav2 empfiehlt/zeigt den Start des **async** Nodes über das Launchfile:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### 3.1 Parameter-Strategie (robust & PR-freundlich)

- Lege **deine** Param-Datei ins Repo:

  - `ros2_ws/src/amr_bringup/config/slam_toolbox_online_async.yaml`
- Starte Launch mit Param-Datei (Pattern, je nach Launchfile/Override-Möglichkeit):

  - entweder per Launch-Arg (wenn vorhanden) oder
  - über eigenes Bringup-Launch in `amr_bringup` (empfohlen ab jetzt)

**Minimal-Overrides**, die du fast immer brauchst:

- `scan_topic: /scan`
- `map_frame: map`
- `odom_frame: odom`
- `base_frame: base_footprint` (oder `base_link`, aber konsistent bleiben)

> Praxis: Starte zunächst mit Standard-Launch, prüfe `ros2 param list`/`ros2 param dump` am Node und ziehe dann eine saubere `amr_bringup`-Launch-Kapsel nach.

---

## 4) Bringup-Launch (empfohlen für dein Repo)

### 4.1 Struktur

```text
ros2_ws/src/amr_bringup/
├─ config/
│  └─ slam_toolbox_online_async.yaml
└─ launch/
   └─ slam_toolbox.launch.py
```

### 4.2 Start (Beispiel)

```bash
ros2 launch amr_bringup slam_toolbox.launch.py
```

Kriterien für ein sauberes Launch:

- Node-Name eindeutig (z. B. `slam_toolbox`)
- Parameterdatei aus dem Repo geladen
- Remaps dokumentiert (falls `/odometry/filtered` → `/odom` o.ä.)

---

## 5) Smoke-Tests (10–15 Minuten)

### 5.1 SLAM läuft wirklich?

```bash
ros2 topic list | egrep "/scan|/map|/tf"
ros2 topic hz /scan
ros2 topic hz /map
```

### 5.2 TF-Kette vollständig?

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

### 5.3 RViz2 Minimal-Check

- Fixed Frame: `map`
- Displays:

  - Map (`/map`)
  - LaserScan (`/scan`)
  - TF

**Erwartung:** Scan liegt „auf“ der Map, keine „schwimmende“ Laserwolke.

---

## 6) Map speichern (für Phase 6 / Nav2)

Nav2 nutzt zum Speichern typischerweise `map_saver_cli`:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/amr_map
```

### Docker-Hinweis (wichtig)

Wenn du im Container speicherst, musst du das Zielverzeichnis als Volume haben, z. B.:

- Host: `amr-platform/maps/`
- Container: `/maps`

Compose-Idee:

```yaml
volumes:
  - ../maps:/maps:rw
```

Dann speichern:

```bash
ros2 run nav2_map_server map_saver_cli -f /maps/amr_map
```

Erwartete Dateien:

- `/maps/amr_map.yaml`
- `/maps/amr_map.pgm`

---

## 7) Typische Fehlerbilder (schnell eingrenzen)

### 7.1 `/map` bleibt leer / keine Updates

- `/scan` kommt nicht oder `scan_topic` falsch
- TF `odom → base_*` fehlt oder ist instabil
- `frame_id` im `/scan` passt nicht zum TF-Baum (`base_laser` muss existieren)

Checks:

```bash
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo base_link base_laser
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 7.2 „Map driftet weg“ / „Scan klebt am Roboter“

- `map→odom` fehlt (SLAM läuft nicht korrekt) oder wird von etwas anderem überschrieben
- `odom→base_*` wird doppelt gepublisht (Bridge + EKF gleichzeitig)

Check:

```bash
ros2 run tf2_tools view_frames
# und prüfen, wer /tf publisht:
ros2 topic info /tf
```

### 7.3 Mapping wirkt „verzogen“

- Wheel-Odom Skalierung (Ticks/m) noch nicht kalibriert
- LiDAR-Mount (yaw) im URDF falsch
- Zu schnelle Bewegung beim Mapping (erst langsam, dann schneller)

---

## 8) Übergang zu Phase 6 (Nav2)

Für Nav2 im SLAM-Betrieb gilt:

- SLAM stellt `/map` + `map→odom`.
- Nav2 wird ohne `map_server`/`amcl` gestartet (weil SLAM die Map liefert).
- Navigation benötigt weiterhin `/cmd_vel`-Pipeline und stabile Odom (Phase 4).

---

## Changelog

- v1.0 (2025-12-19): Online-Async Mapping mit SLAM Toolbox, Smoke-Tests, Map-Save-Workflow (Docker-Volumes), klare TF-Kontrakte für Phase 6.
