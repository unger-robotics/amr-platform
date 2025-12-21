# Phase 4: TF-Baum – Befehlsreferenz

**Status:** In Arbeit  
**Version:** 2.2

---

## 1) Dateien kopieren (Mac)

```bash
cd ~/daten/start/IoT/AMR/amr-platform

# ZIP entpacken (ueberschreibt bestehende Dateien)
unzip -o phase4-ros2-packages.zip

# Struktur pruefen
ls -la ros2_ws/src/amr_bridge/
ls -la ros2_ws/src/amr_description/
ls -la ros2_ws/src/amr_bringup/
```

### Struktur nach dem Kopieren

```
ros2_ws/src/
├── amr_bridge/
│   ├── amr_bridge/
│   │   ├── __init__.py
│   │   └── odom_converter.py
│   ├── resource/
│   │   └── amr_bridge
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── amr_bringup/
│   ├── launch/
│   │   └── amr.launch.py       # NEU: Startet alles
│   ├── package.xml
│   └── CMakeLists.txt
├── amr_description/
│   ├── urdf/
│   │   └── amr.urdf
│   ├── launch/
│   │   └── description.launch.py
│   ├── package.xml
│   └── CMakeLists.txt
└── sllidar_ros2/                # bereits vorhanden
```

---

## 2) Git Push (Mac)

```bash
cd ~/daten/start/IoT/AMR/amr-platform
git add .
git commit -m "Phase 4: TF-Baum (odom_converter, URDF, Launch)"
git push origin main
```

---

## 3) Git Pull & Docker Rebuild (Pi)

```bash
ssh pi@rover
cd ~/amr-platform
git pull origin main

# Docker-Image neu bauen (einmalig fuer Phase 4)
cd ~/amr-platform/docker
docker compose down
docker compose build --no-cache
docker compose up -d

# In Container wechseln
docker compose exec amr_dev bash
```

Im Container:

```bash
source /opt/ros/humble/setup.bash
cd /root/ros2_ws

# Alle AMR-Packages bauen
colcon build --packages-select amr_bridge amr_description amr_bringup
source install/setup.bash
```

---

## 4) Starten

### Option A: Alles mit einem Befehl (empfohlen)

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Startet: RPLidar + robot_state_publisher + odom_converter
ros2 launch amr_bringup amr.launch.py

# Falls anderer USB-Port:
ros2 launch amr_bringup amr.launch.py serial_port:=/dev/ttyUSB1
```

### Option B: Einzeln starten (Debugging)

**Terminal 1: RPLidar**
```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 2: TF-Baum**
```bash
ros2 launch amr_description description.launch.py
```

---

## 5) Smoke-Tests

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Topics pruefen
ros2 topic list
# Erwartung: /odom, /odom_raw, /scan, /tf, /tf_static

# /odom vorhanden?
ros2 topic echo /odom --once

# TF-Baum pruefen
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom laser

# TF-Baum als PDF
ros2 run tf2_tools view_frames
# Erzeugt: frames_<timestamp>.pdf
```

---

## 6) Erwartete Topics

```
/cmd_vel           # Teleop-Befehle
/esp32/heartbeat   # ESP32 Status
/esp32/led_cmd     # LED-Steuerung
/odom_raw          # ESP32 (Pose2D)
/odom              # NEU: odom_converter (Odometry)
/scan              # RPLidar
/tf                # NEU: Dynamische TFs
/tf_static         # NEU: Statische TFs (URDF)
/robot_description # NEU: URDF
```

---

## 7) Erwarteter TF-Baum

```
odom (dynamisch: odom_converter)
  │
  └── base_footprint
        │
        └── base_link (statisch: URDF)
              │
              └── laser (statisch: URDF)
```

---

## 8) Troubleshooting

| Problem | Loesung |
|---------|---------|
| "Package not found" | `colcon build` wiederholen, `source install/setup.bash` |
| TF disconnected | Alle Nodes laufen? `ros2 node list` pruefen |
| Laser schwebt/falsch | URDF-Masse anpassen (siehe unten) |
| Kein /odom | odom_converter laeuft nicht, ESP32 sendet nicht |
| Kein /odom_raw | micro-ROS Agent laeuft nicht |

---

## 9) URDF-Masse anpassen

Falls der Laser in RViz2 falsch positioniert ist:

```bash
# Auf Mac editieren
nano ros2_ws/src/amr_description/urdf/amr.urdf
```

Relevante Zeilen:

```xml
<!-- base_link Hoehe ueber Boden (Rad-Radius + halbe Chassis-Hoehe) -->
<origin xyz="0 0 0.05" rpy="0 0 0"/>

<!-- Laser Position relativ zu base_link -->
<!-- x: vor/hinter Mitte, y: links/rechts, z: ueber base_link -->
<origin xyz="0.08 0 0.08" rpy="0 0 0"/>
```

Nach Aenderung: Git push, Pi pull, rebuild.

---

## 10) Definition of Done (DoD)

- [ ] `ros2 topic list` zeigt /odom, /tf, /tf_static
- [ ] `ros2 run tf2_ros tf2_echo odom laser` liefert kontinuierliche Werte
- [ ] `ros2 run tf2_tools view_frames` erzeugt zusammenhaengenden Baum
- [ ] RViz2: RobotModel + LaserScan korrekt relativ zueinander

---

## 11) Naechste Phase

Phase 4 ist abgeschlossen, wenn:
- /scan im Frame `laser` ankommt
- `tf2_echo odom laser` stabil ist
- RViz RobotModel + LaserScan deckungsgleich sind

Dann: **Phase 5 (SLAM)** starten.
