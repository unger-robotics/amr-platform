# To-Do-Liste AMR-Projekt

> **Stand:** 2025-12-21
> **To-Do-Regel:** Dieses Dokument ist eine **Aufgabenliste** (Was fehlt noch?).

---

## Projektstatus (kurz)

| Phase | Beschreibung | Status |
|------:|--------------|:------:|
| 1 | micro-ROS auf ESP32-S3 | ✅ |
| 2 | Docker-Infrastruktur | ✅ |
| 3 | RPLidar A1 Integration | ✅ |
| 4 | URDF + TF + EKF | ◄── **als Nächstes** |
| 5 | SLAM (slam_toolbox) | ⬜ |
| 6 | Nav2 Autonome Navigation | ⬜ |
| 7 | Vision/AI (Hailo) + Servo-Pan | ⬜ |

---

## To-Dos (nach Phasen gruppiert)

### Phase 4 — URDF + TF + EKF (als Nächstes)

**Ziel:** Konsistenter TF-Baum + saubere Odometrie-Grundlage für SLAM/Nav2.

#### 4.1 URDF + TF-Grundgerüst

- [ ] URDF-Grundgerüst anlegen
  - [ ] `base_footprint` (2D, yaw-only)
  - [ ] `base_link` (3D Body-Frame)
  - [ ] Sensor-Frame festlegen und einheitlich benennen:
    - [ ] LiDAR: `laser` **oder** `laser_frame` (Entscheidung dokumentieren)
- [ ] `robot_state_publisher` einbinden (TF aus URDF)
- [ ] Statische TFs prüfen (z. B. `base_link -> laser`)

**Ziel-Frames (Projektziel, empfohlen):**

```text
map -> odom -> base_footprint -> base_link -> laser
```

#### 4.2 Odometrie-Bridge (wenn nötig)

- [ ] `odom_converter.py` (oder Node) hinzufügen

  - [ ] `/odom_raw` → `/odom` (nav_msgs/Odometry)
  - [ ] TF publizieren: `odom -> base_footprint` (nur wenn nicht schon von EKF geliefert)

**Regel:** Genau **eine** Instanz im System darf `odom -> base_*` als TF publizieren (sonst TF-Konflikte).

#### 4.3 EKF (robot_localization)

- [ ] `robot_localization` integrieren
- [ ] EKF-Config-Datei anlegen

  - [ ] Inputs festlegen (Wheel Odom, optional IMU später)
  - [ ] Output-Topic definieren (typisch `/odometry/filtered`)
  - [ ] Output-TF-Regel festlegen:

    - [ ] EKF publiziert `odom -> base_footprint` **oder**
    - [ ] EKF publiziert keinen TF und Bridge übernimmt (einheitlich entscheiden)
- [ ] Frame-IDs prüfen (`odom`, `base_footprint`, `base_link`, `laser`)

#### 4.4 Smoke-Tests (Phase 4)

- [ ] `ros2 topic list` enthält: `/tf`, `/tf_static`, `/odom` (oder `/odometry/filtered`)
- [ ] TF-Echo:

  - [ ] `ros2 run tf2_ros tf2_echo odom base_footprint` liefert Daten
  - [ ] `ros2 run tf2_ros tf2_echo base_link laser` liefert statische TF
- [ ] Odometrie:

  - [ ] `ros2 topic hz /odom` (oder `/odometry/filtered`) stabil

---

### Phase 5 — SLAM

- [ ] `slam_toolbox` installieren/integrieren
- [ ] Online SLAM (async) starten
- [ ] Testraum abfahren und Karte erzeugen
- [ ] Karte speichern (`.yaml` + `.pgm`/`.png`)
- [ ] Re-Start mit gespeicherter Karte getestet

---

### Phase 6 — Nav2

- [ ] Nav2-Stack integrieren
- [ ] Lokalisierung einrichten (z. B. AMCL **oder** slam_toolbox localization mode)
- [ ] Costmaps konfigurieren (footprint, inflation, obstacle layers)
- [ ] Autonome Navigation testen (Goal setzen, fährt reproduzierbar)
- [ ] Recovery/Timeouts prüfen (Stop/Cancel funktioniert)

---

### Phase 7 — Vision/AI (Hailo) + Servo-Pan (optional)

- [ ] Kamera bringup (`/camera/image_raw`, `/camera/camera_info`)
- [ ] Hailo-Inferenz bringup (`/vision/detections`)
- [ ] Vision-Events definieren (`/vision/stop_request`, `/vision/slowdown_factor`)
- [ ] Integration in Fahrstack (Velocity-Gate / Stop-Trigger), ohne LiDAR-Safety zu ersetzen
- [ ] MG90S Pan-Servo (optional):

  - [ ] Separate 5V-Servo-Versorgung + gemeinsame Masse
  - [ ] `/pan/command` steuert Winkel reproduzierbar

---

## Hardware/Ports (Ist-Zustand)

| Device          | Port               | Zweck                                  |
| --------------- | ------------------ | -------------------------------------- |
| ESP32-S3        | `/dev/ttyACM0`     | micro-ROS (921600 Baud)                |
| RPLidar A1      | `/dev/ttyUSB0`     | LaserScan                              |
| Pi Camera (CSI) | (kein `/dev/tty*`) | Zugriff über libcamera/ROS Kamera-Node |
| Hailo (AI Kit)  | (systemabhängig)   | Zugriff über Hailo Runtime / ROS Node  |

---

## Quick Reference (Arbeitsbefehle)

### Container starten

```bash
cd ~/amr-platform/docker
docker compose up -d
```

### Dev-Shell im Container

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

### RPLidar starten

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

### Topics prüfen

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /scan --once
```
