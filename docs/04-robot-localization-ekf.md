---
title: "Phase 4 – robot_localization (EKF): IMU + Wheel/Odom → stabiles odom"
file: "docs/roadmap/04-robot-localization-ekf.md"
version: "v0.1.0"
status: "draft"
scope: "AMR Projekt-Nachschlagewerk (Master-grade Design, privat)"
---

# Phase 4 – robot_localization (EKF): IMU + Wheel/Odom → stabiles `odom`

`robot_localization` liefert eine **gefilterte, lokal konsistente Odometrie** aus mehreren Sensorquellen (typisch: Encoder-Odom + IMU) und kann zusätzlich den **TF `odom → base_*`** publizieren. :contentReference[oaicite:0]{index=0}

**Abhängigkeiten (vorher):**

- Phase 1: micro-ROS auf ESP32-S3 (Odom/Encoder, Motor)
- Phase 2: ROS 2 Jazzy auf Pi 5 (Docker) + micro-ROS Agent + Bridge-Smoke-Tests
- Phase 2.5: URDF + TF Frames (saubere Frame-Namen & REP-103-konform)

---

## 1. Zielbild & Definition of Done (DoD)

**Ziel:** Nav2/SLAM bekommen ein **ruhiges, konsistentes** `odom`, bevor globale Lokalisierung (map) dazukommt. :contentReference[oaicite:1]{index=1}

**DoD (messbar):**

- `robot_localization` publiziert:
  - `/odometry/filtered` (oder remapped auf `/odom`)
  - optional `/tf` mit `odom → base_footprint` (oder `odom → base_link`) :contentReference[oaicite:2]{index=2}
- Update-Rate: `>= 30\,\mathrm{Hz}` (z. B. `50\,\mathrm{Hz}`), keine “Stotter”-Aussetzer > `0{,}2\,\mathrm{s}`
- In `rviz2`: TF-Baum stabil, keine Frame-Sprünge bei Geradeausfahrt
- Encoder-Odom driftet weiterhin (physikalisch normal), aber:
  - Heading (Yaw) wird durch IMU-Yaw-Rate sichtbar ruhiger
  - Twist/Velocity ist weniger verrauscht (spürbar “smooth”)

---

## 2. Datenvertrag (Topics, Frames, Einheiten)

### 2.1 REP-103 (wichtig für späteres Nav2/SLAM)

- Körperfest: `x` vorwärts, `y` links, `z` oben :contentReference[oaicite:3]{index=3}
- Welt-nah (typisch ROS): ENU (`X` east, `Y` north, `Z` up) :contentReference[oaicite:4]{index=4}

### 2.2 Erwartete Inputs (Minimal-Setup)

- Wheel/Odom: `nav_msgs/Odometry`
  **Topic:** z. B. `/wheel/odom` oder `/amr/odom`
  **frames:** `header.frame_id = "odom"`, `child_frame_id = "base_footprint"` (oder `"base_link"`)
- IMU: `sensor_msgs/Imu`
  **Topic:** `/imu/data`
  **frame:** `header.frame_id = "imu_link"` (URDF fix an `base_link`)

**Muss:** Zeitstempel laufen sauber vorwärts; Covariances sind gesetzt (nicht alles 0). (robot_localization gewichtet Sensoren über Kovarianzen; “0” wirkt wie “perfekt” und macht Fusion instabil.)

---

## 3. Architektur-Entscheidung: `world_frame = odom` (Phase 4)

In Phase 4 fusionierst du **kontinuierliche** Sensorsignale (Encoder/IMU). Dafür ist der Standard:

- `world_frame = odom`
- EKF publiziert `odom → base_*` :contentReference[oaicite:5]{index=5}

**Wichtig (für später):** Wenn du irgendwann globale, sprunghafte Daten (SLAM/GPS/AMCL) fusionierst, gilt eine andere Regel: `world_frame = map` und **etwas anderes** muss `odom → base_*` liefern (z. B. zweiter EKF ohne globale Daten). :contentReference[oaicite:6]{index=6}

---

## 4. Konfiguration (empfohlen): planar, robust, nav2-tauglich

### 4.1 Dateiablage (PR-freundlich)

- `ros2_ws/src/amr_bringup/config/ekf_odom.yaml`
- `ros2_ws/src/amr_bringup/launch/ekf_odom.launch.py` (optional, aber sauber)

### 4.2 `ekf_odom.yaml` (Startkonfiguration)

Diese Vorlage orientiert sich an der Nav2-Empfehlung (Parameter, Frames, Sensor-Config-Arrays). :contentReference[oaicite:7]{index=7}

> **Hinweis zum YAML-Top-Key:** Der oberste Key muss zum Node-Namen aus dem Launch passen (üblich: `ekf_filter_node`). In Nav2-Beispielen ist das genauso gezeigt. :contentReference[oaicite:8]{index=8}

```yaml
### EKF (odom) – IMU + Wheel/Odom → odometry/filtered + optional TF odom->base_*
ekf_filter_node:
  ros__parameters:
    # Output-Rate des Filters (läuft kontinuierlich, sobald erste Messung da ist)
    frequency: 50.0

    # Planar fahren: ignoriere z, roll, pitch (reduziert IMU-Bodenwellen-Effekte)
    two_d_mode: true

    # /tf: odom -> base_* aus robot_localization (Phase 4 so gewollt)
    publish_tf: true

    # Frames (Phase 2.5 muss dazu passen)
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint   # alternativ: base_link
    world_frame: odom                 # kontin. Sensorfusion → odom (Standard)

    # -----------------------
    # Input 0: Wheel Odometry
    # -----------------------
    odom0: /wheel/odom

    # 15er-Config-Vektor:
    # [x, y, z,
    #  roll, pitch, yaw,
    #  vx, vy, vz,
    #  vroll, vpitch, vyaw,
    #  ax, ay, az]
    #
    # Empfehlung für diff-drive Start:
    # - nutze Twist (vx, vyaw)
    # - optional: vy (bei Mecanum/Omni), sonst false
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    # -----------------------
    # Input 0: IMU
    # -----------------------
    imu0: /imu/data

    # Minimal robust (ohne Magnetometer): nutze nur vyaw (Yaw-Rate)
    # (entspricht dem Nav2-Beispielansatz) :contentReference[oaicite:9]{index=9}
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  false, false, false]
```

**Wenn dein IMU-Heading wirklich stabil ist** (z. B. IMU mit Magnetometer + guter Fusion), kannst du später zusätzlich `yaw` aktivieren – aber erst, wenn du Drift/Offsets verstanden hast.

---

## 5. Launch / Start (Docker auf Pi 5)

### 5.1 Installation im Jazzy-Container

`robot_localization` per apt im Container installieren (ros-<distro>-robot-localization ist der offizielle Weg). ([Nav2 Dokumentation][1])

### 5.2 Start (Beispiel ohne Launchfile)

```bash
ros2 run robot_localization ekf_node --ros-args --params-file \
  ~/ros2_ws/src/amr_bringup/config/ekf_odom.yaml
```

### 5.3 Start (Launch-Pattern, empfohlen)

Nav2 zeigt das Launch-Pattern mit `executable='ekf_node'` + `name='ekf_filter_node'` + params-file. ([Nav2 Dokumentation][1])

---

## 6. Smoke-Tests (10 Minuten)

### 6.1 Topics vorhanden?

`robot_localization` publiziert typischerweise `odometry/filtered` und optional `/tf`. ([Nav2 Dokumentation][1])

```bash
ros2 topic list | egrep "odometry/filtered|/tf|wheel/odom|imu/data"
ros2 topic hz /odometry/filtered
```

### 6.2 EKF subscribed wirklich?

```bash
ros2 node info /ekf_filter_node
```

Du willst `/wheel/odom` und `/imu/data` als Subscribers sehen.

### 6.3 TF korrekt?

Nav2 empfiehlt `tf2_echo` auf `odom base_link` (bei dir ggf. `base_footprint`). ([Nav2 Dokumentation][1])

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 6.4 Sanity: Frames & Einheiten

- Geradeausfahrt: `x` steigt, `y` ~0, yaw ändert sich wenig (REP-103 beachten). ([ros.org][2])
- Drehung auf der Stelle: `vyaw` != 0, `vx` ~0

---

## 7. Häufige Fehlerbilder (gezielt)

1. **TF doppelt** (flackert / springt)
   Ursache: Noch ein anderer Node publiziert `odom → base_*` (z. B. Bridge, Gazebo, eigener broadcaster).
   Fix: Nur **eine** Quelle für diesen TF. Nav2 weist explizit darauf hin, TF-Bridges zu entfernen, wenn EKF TF publizieren soll. ([Nav2 Dokumentation][1])

2. **Filter läuft nicht an**
   Ursache: EKF startet erst, wenn mindestens ein Input empfangen wurde. ([Nav2 Dokumentation][1])
   Fix: Prüfe, ob `/wheel/odom` und `/imu/data` wirklich Daten liefern.

3. **Falsches Vorzeichen / Achsen vertauscht**
   Ursache: Nicht REP-103-konforme Frames (z. B. `x` rechts statt vorwärts).
   Fix: URDF/TF-Frames korrigieren (Phase 2.5). ([ros.org][2])

4. **“Klebt” / driftet stark**
   Ursache: Odom/IMU Covariance nicht sinnvoll (z. B. alles 0).
   Fix: realistische Kovarianzen setzen (oder zumindest nicht 0).

---

## 8. Output-Integration für Phase 5/6 (SLAM / Nav2)

Empfehlung:

- **Input für SLAM/Nav2:** `/odometry/filtered` (oder remap → `/odom`)
- **TF:** `odom → base_footprint` aus EKF (Phase 4)
- **map-Frame kommt erst später** durch SLAM/AMCL; EKF bleibt auf `world_frame=odom` (Phase 4 Logik). ([Nav2 Dokumentation][1])

---

## 9. Mini-Checkliste (PR-ready)

- [ ] `ekf_odom.yaml` im Repo (config/)
- [ ] `odom_frame`, `base_link_frame`, `imu_link` passen zu Phase 2.5 URDF/TF
- [ ] `/wheel/odom` und `/imu/data` liefern valide Stamps + Covariances
- [ ] `ros2 topic hz /odometry/filtered` stabil (>= `30\,\mathrm{Hz}`)
- [ ] `tf2_echo odom base_footprint` stabil (ohne Sprünge)

---

## Quellen (primär)

- Nav2 Guide: “Smoothing Odometry using Robot Localization” (Frames, `publish_tf`, `two_d_mode`, Config-Arrays, Launch-Pattern). ([Nav2 Dokumentation][1])
- ROS REP-103: Koordinatenkonventionen (`x` forward, `y` left, `z` up). ([ros.org][2])
- ROS Index / robot_localization: Paketüberblick. ([ROS Index][3])

```
::contentReference[oaicite:22]{index=22}
```

[1]: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html "Smoothing Odometry using Robot Localization — Nav2 1.0.0 documentation"
[2]: https://www.ros.org/reps/rep-0103.html?utm_source=chatgpt.com "REP 103 -- Standard Units of Measure and Coordinate ..."
[3]: https://index.ros.org/p/robot_localization/?utm_source=chatgpt.com "robot_localization - ROS Package Overview"
