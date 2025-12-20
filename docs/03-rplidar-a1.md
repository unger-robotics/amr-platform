---
title: "Phase 3 – RPLidar A1 Integration + /scan + RViz2"
status: "active"
updated: "2025-12-19"
depends_on:
  - "Phase 2 (ROS 2 Jazzy via Docker)"
next:
  - "Phase 4 (robot_localization EKF)"
---

# Phase 3: RPLidar A1 Integration (+ /scan + RViz2)

## Zielbild & Definition of Done

### Zielbild

- RPLidar A1 läuft am Raspberry Pi 5 (Docker/ROS 2 Jazzy) als Laser-Treiber.
- Topic `/scan` (`sensor_msgs/msg/LaserScan`) ist stabil und hat plausible Werte.
- Frame-ID des Scans ist **`base_laser`** (passend zu Phase 2.5).
- RViz2 zeigt `/scan` korrekt relativ zu `base_link`/`base_footprint`.

### DoD (prüfbar)

- [ ] `ros2 topic hz /scan` zeigt stabile Frequenz (typisch ~`5.5\,\mathrm{Hz}`, je nach Setting/Modell). :contentReference[oaicite:0]{index=0}
- [ ] `ros2 topic echo /scan --once` liefert plausible `range_min/range_max` und echte Messpunkte.
- [ ] `ros2 run tf2_ros tf2_echo base_link base_laser` funktioniert (TF vorhanden).
- [ ] RViz2: LaserScan sichtbar, keine TF-Warnungen, Scan rotiert nicht „gegenläufig“.

---

## 1) Hardware & Basisdaten (Regel)

### 1.1 Erwartete Sensor-Eckdaten (für Plausibilitätschecks)

- Reichweite: bis ca. `12\,\mathrm{m}` (modellabhängig; einige A1-Varianten effektiv niedriger). :contentReference[oaicite:1]{index=1}
- Scanrate: typisch `5.5\,\mathrm{Hz}` bei ca. 1450 Punkten/Umdrehung; konfigurierbar bis `10\,\mathrm{Hz}`. :contentReference[oaicite:2]{index=2}

### 1.2 Anschluss

- RPLidar A1 über USB-Adapter an Pi 5.
- Serial-Baudrate hängt am Adapter/Modell; A1-Adapter sind häufig `115200\,\mathrm{bps}`. :contentReference[oaicite:3]{index=3}

---

## 2) Serial-Device stabil machen (Beobachtung → Daten)

**Regel:** nutze `/dev/serial/by-id/...` statt `/dev/ttyUSB0`, damit Reboots/Umstecken nicht brechen.

```bash
ls -l /dev/serial/by-id/
dmesg | tail -n 50
```

Erwartung: ein Eintrag wie `... -> ../../ttyUSB0` oder `ttyACM0`.

---

## 3) Treiber-Entscheidung

### Option A (empfohlen): `sllidar_ros2` (Slamtec ROS 2 Driver)

- Vorteil: ROS2-native Launchfiles inkl. A1 + RViz. ([GitHub][1])

### Option B: `rplidar_ros` (ROS2 branch)

- Vorteil: etabliert, ROS-Doku/Index vorhanden; A1 Launchfile verfügbar. ([docs.ros.org][2])

**Für Zukunftssicherheit:** Option A ist der „direkte“ ROS2-Weg; beide sind nutzbar, aber starte mit A.

---

## 4) Docker/Compose: Device in Container durchreichen (Anwendung)

### 4.1 Minimal: Device-Passthrough per by-id

In deinem `docker-compose.yml` (Service, der den Lidar-Node startet) zusätzlich:

```yaml
devices:
  - /dev/serial/by-id:/dev/serial/by-id
```

Pragmatisch für Phase 3: Container als root laufen lassen (keine Dialout-Group-Diskussion).

---

## 5) Installation im Workspace (Option A: sllidar_ros2)

Im ROS-Container (Jazzy):

```bash
cd /ws
mkdir -p src
cd src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Das Repo dokumentiert Build/Run und Launch für A1. ([GitHub][1])

---

## 6) Start: A1 + RViz2

### 6.1 Quickstart-Launch (A1)

```bash
# im ROS-Container
source /ws/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

Der A1-Launch ist im offiziellen sllidar_ros2 README beschrieben. ([GitHub][1])

### 6.2 Wichtige Parameter (Minimal)

Ziel: **`frame_id = base_laser`**.

Prüfe in der Launch-Datei (oder via Override), dass `frame_id` passt:

- `frame_id: base_laser`
- `serial_port: /dev/serial/by-id/<DEIN-LIDAR>`
- `serial_baudrate: 115200` (wenn A1-Adapter typisch) ([Slamtec Wiki][3])

---

## 7) Smoke-Tests (muss grün sein)

### 7.1 Topic lebt und hat Rate

```bash
ros2 topic list | grep scan
ros2 topic hz /scan
```

### 7.2 Inhalt plausibel

```bash
ros2 topic echo /scan --once
```

Plausibilitätscheck:

- `angle_min` nahe `-\pi`, `angle_max` nahe `+\pi`
- `ranges[]` enthält Werte im Bereich `range_min..range_max`, „0.0“/`inf` sind bei Nicht-Reflexion normal.

### 7.3 TF passt (Phase 2.5 vorausgesetzt)

```bash
ros2 run tf2_ros tf2_echo base_link base_laser
```

Wenn Phase 2.5 noch nicht aktiv ist, setze temporär einen Static TF:

```bash
ros2 run tf2_ros static_transform_publisher \
  0.12 0.00 0.15 0 0 0 base_link base_laser
```

(Offsets sind Platzhalter; echte Maße später in URDF übernehmen.)

---

## 8) RViz2 Setup (minimal, reproduzierbar)

1. `Fixed Frame` auf `base_link` (oder `odom`, sobald vorhanden).
2. Display hinzufügen:

   - **LaserScan**
   - Topic: `/scan`
3. Optional:

   - **TF** Display aktivieren (Frames sichtbar machen)

Erwartung: Scan sitzt am Sensor und dreht korrekt (keine Spiegelung).

---

## 9) Troubleshooting (typische Ursachen)

### Kein `/scan`

- falscher Serial-Port (by-id prüfen)
- falsche Baudrate (A1-Adapter häufig `115200`) ([Slamtec Wiki][3])
- Device nicht im Container gemountet (`devices:` fehlt)

### „Permission denied“ auf `/dev/ttyUSB0`

- Quickfix: Container als root laufen lassen
- Alternativ: Host udev rule + Gruppe `dialout` (Slamtec-Repos liefern Skripte/Anleitung). ([GitHub][1])

### Scan „steht“ oder ist extrem langsam

- Unterspannung am Pi (USB-Power/Hub prüfen)
- falsche Scan-Frequenz/Mode (später feinjustieren; Phase 3: erstmal stabil)

### Scan wirkt in RViz2 verdreht

- `frame_id` nicht `base_laser`
- `base_link -> base_laser` yaw/Rotation falsch (URDF/Static TF korrigieren)

---

## Changelog

- v1.0 (2025-12-19): Phase-3 Ablauf mit sllidar_ros2 (A1 Launch), Docker-Device-Passthrough, Smoke-Tests und RViz2/TF-Kriterien. ([GitHub][1])
