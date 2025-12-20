---
title: "Phase 4 – URDF + TF Frames + EKF Sensor Fusion"
status: "planned"
updated: "2025-12-20"
version: "2.0"
depends_on:
  - "Phase 1 (micro-ROS ESP32-S3) ✅"
  - "Phase 2 (Docker-Infrastruktur) ✅"
  - "Phase 3 (RPLidar A1)"
next:
  - "Phase 5 (SLAM)"
---

# Phase 4: URDF + TF Frames + EKF Sensor Fusion

## Zielbild & Definition of Done

### Zielbild (TF-Baum)

```
map → odom → base_footprint → base_link → base_laser
                                       → imu_link (optional)
```

### DoD (prüfbar)

- [ ] `robot_state_publisher` publiziert **statische TFs** aus URDF
- [ ] EKF publiziert **dynamische TF** `odom → base_footprint`
- [ ] `tf2_tools/view_frames` erzeugt konsistenten Baum
- [ ] RViz2 zeigt Robot-Modell + TF + LaserScan korrekt

---

## 1) Voraussetzungen

### 1.1 Abgeschlossene Phasen

| Phase | Status | Liefert |
|-------|--------|---------|
| Phase 1 | ✅ | `/odom_raw` (Pose2D), `/cmd_vel` |
| Phase 2 | ✅ | Docker-Container, ROS 2 Humble |
| Phase 3 | 🔜 | `/scan` (LaserScan) |

### 1.2 Benötigte Packages

```bash
# Im Container installieren
apt-get install -y \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-tools
```

---

## 2) Konventionen (REP-103 / REP-105)

### 2.1 Achsen

| Achse | Richtung |
|-------|----------|
| x | vorwärts |
| y | links |
| z | oben |

### 2.2 Frame-Rollen

| Frame | Rolle | Quelle |
|-------|-------|--------|
| `map` | Globale Karte | SLAM / AMCL |
| `odom` | Lokal kontinuierlich | EKF |
| `base_footprint` | Roboter am Boden (z=0) | URDF (statisch) |
| `base_link` | Körper-Frame | URDF (statisch) |
| `base_laser` | LiDAR Ursprung | URDF (statisch) |
| `imu_link` | IMU Ursprung | URDF (statisch) |

**Regel:**

- URDF liefert **statische** TFs (link → link)
- EKF liefert **dynamische** TF (odom → base_footprint)
- SLAM/AMCL liefert **dynamische** TF (map → odom)

---

## 3) Messungen (TODO)

### 3.1 Ursprung definieren

- `base_footprint`: Mittelpunkt zwischen Radaufstandspunkten, z=0
- `base_link`: Chassis-Ebene (z-Offset über base_footprint)

### 3.2 Zu messende Abstände

| Offset | Wert (TODO) | Beschreibung |
|--------|-------------|--------------|
| `base_link_z` | ? m | Höhe base_link über Boden |
| `laser_x` | ? m | LiDAR vor/hinter base_link |
| `laser_y` | 0 m | LiDAR links/rechts |
| `laser_z` | ? m | LiDAR über base_link |
| `laser_yaw` | 0 oder π | LiDAR Orientierung |

---

## 4) URDF/Xacro-Struktur

### 4.1 Dateistruktur

```
ros2_ws/src/amr_description/
├── urdf/
│   └── amr.urdf.xacro
├── launch/
│   └── description.launch.py
└── rviz/
    └── amr.rviz
```

### 4.2 Minimal-URDF

```xml
<?xml version="1.0"?>
<robot name="amr" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Parameter (TODO: messen) -->
  <xacro:property name="base_link_z" value="0.08"/>
  <xacro:property name="laser_x" value="0.12"/>
  <xacro:property name="laser_y" value="0.00"/>
  <xacro:property name="laser_z" value="0.15"/>

  <!-- Links -->
  <link name="base_footprint"/>
  <link name="base_link"/>
  <link name="base_laser"/>

  <!-- Joints (statisch) -->
  <joint name="base_footprint_to_base_link" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${base_link_z}" rpy="0 0 0"/>
  </joint>

  <joint name="base_link_to_base_laser" type="fixed">
    <parent link="base_link"/>
    <child link="base_laser"/>
    <origin xyz="${laser_x} ${laser_y} ${laser_z}" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## 5) Odom Converter (Bridge Node)

### 5.1 Funktion

Konvertiert `/odom_raw` (Pose2D) → `/odom` (Odometry) + TF

### 5.2 Input/Output

| Topic | Typ | Richtung |
|-------|-----|----------|
| `/odom_raw` | `geometry_msgs/Pose2D` | Input |
| `/odom` | `nav_msgs/Odometry` | Output |
| `/tf` | `tf2_msgs/TFMessage` | Output |

### 5.3 TF publiziert

```
odom → base_footprint
```

---

## 6) EKF (robot_localization)

### 6.1 Konfiguration (ekf.yaml)

```yaml
ekf_node:
  ros__parameters:
    frequency: 50.0
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    odom0: /odom
    odom0_config: [true, true, false,   # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   false, false, false,  # vx, vy, vz
                   false, false, false,  # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az

    publish_tf: true
```

### 6.2 Launch

```bash
ros2 launch robot_localization ekf.launch.py
```

---

## 7) Verifikation

### 7.1 TF-Baum prüfen

```bash
# Graph generieren
ros2 run tf2_tools view_frames

# Einzelne Transform prüfen
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link base_laser
```

### 7.2 RViz2

```bash
ros2 run rviz2 rviz2
```

- Add: TF
- Add: LaserScan (Topic: `/scan`, Frame: `base_laser`)
- Add: RobotModel (Topic: `/robot_description`)

---

## 8) Troubleshooting

| Problem | Ursache | Lösung |
|---------|---------|--------|
| TF "disconnected" | robot_state_publisher fehlt | Node starten |
| Laser zeigt falsch | yaw im Joint falsch | rpy anpassen |
| Kein odom→base_footprint | Bridge/EKF fehlt | Node prüfen |
| base_footprint schwebt | z-Offset falsch | base_link_z messen |

---

## 9) Abhängigkeit von Phase 3

Phase 4 kann erst vollständig getestet werden, wenn:

1. **Phase 3** `/scan` publiziert
2. LaserScan in RViz2 mit korrektem Frame darstellbar
3. TF-Baum komplett (inkl. `base_laser`)

---

## 10) Changelog

| Version | Datum | Änderungen |
|---------|-------|------------|
| v2.0 | 2025-12-20 | Umbenannt zu Phase 4, Abhängigkeiten aktualisiert, EKF hinzugefügt |
| v1.0 | 2025-12-19 | Initiale Version als Phase 2.5 |
