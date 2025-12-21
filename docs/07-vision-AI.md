---
title: "Phase 7 – Vision/AI Autonomes Fahren – Objekterkennung"
type: "phase-doc"
file: "docs/07-vision-AI.md"
version: "v0.2.0"
status: "draft"
updated: "2025-12-21"
scope: "Projekt-Nachschlagewerk für den AMR-Betrieb (Engineering-Referenz, kein Hochschul-Skript)"
depends_on:
  - "Phase 2 (Docker ROS 2 Humble auf Pi 5) ✅"
  - "Phase 3 (RPLidar A1 + /scan) ✅"
  - "Phase 4 (URDF + TF + /odom) ⬜"
  - "Phase 5 (SLAM: /map + map→odom ODER gespeicherte Map + AMCL) ⬜"
  - "Phase 6 (Nav2: Navigation auf Karte (map)) ⬜"
next:
  - "Phase 7 Vision/AI Autonomes Fahren – Objekterkennung"
---

# Phase 7 – Vision/AI Autonomes Fahren – Objekterkennung

## 0) Ziel und Regel der Phasen-Dokumentation

### Ziel

Vision ergänzt den AMR um **Semantik** (z. B. „Person“, „Objektklasse“, „Markierung“) und erzeugt daraus **stabile Events** für den Fahrstack (Stop/Slowdown).
Die Inferenz läuft hardwarebeschleunigt auf dem **Raspberry Pi AI Kit (Hailo-8L, 13 TOPS)**. :contentReference[oaicite:0]{index=0}

### Regel (Scope-Grenze)

- Vision ist **Assistenzschicht**, nicht die primäre Kollisionsvermeidung (LiDAR bleibt Safety-Basis).
- Kein End-to-End „AI fährt alleine“. Vision liefert **Events**, der Fahrstack (Nav2/Velocity-Gate) entscheidet final.
- Fokus: **Bringup, Datenverträge, Tests, Fehlerbilder, Safety**.

---

## 1) Zielbild (Outcome)

Wenn Phase 7 aktiv ist, gilt:

- Kamera-Stream kommt von der **Raspberry Pi Global Shutter Camera (Sony IMX296, ~1.6 MP, Global Shutter)**. :contentReference[oaicite:1]{index=1}
  Richtwert: **$1456\times1088@60\,\mathrm{fps}$** möglich (je nach Pipeline/Output). :contentReference[oaicite:2]{index=2}
- Objekt-Detektion läuft auf **Hailo-8L** (Edge TPU/Accelerator via PCIe auf Pi 5). :contentReference[oaicite:3]{index=3}
- Optional: Kamera-Pan (MG90S) erlaubt „Target Centering“ (Objekt in Bildmitte halten), ohne dass die Fahrlogik davon abhängt.

---

## 2) Definition of Done (DoD) – prüfbar

- [ ] `rpicam-hello --list-cameras` erkennt die Global-Shutter-Kamera (Pi OS Bookworm). :contentReference[oaicite:4]{index=4}
- [ ] ROS: `image_raw` + `camera_info` stabil:
  - [ ] `/camera/image_raw` $\ge 15\,\mathrm{Hz}$ bei $640\times480$
  - [ ] `/camera/camera_info.header.frame_id == camera_link`
- [ ] Hailo Device verfügbar (Inferenz läuft auf Hailo, nicht CPU-only):
  - [ ] Detector-Node publiziert `/vision/detections` kontinuierlich
  - [ ] Detektionsrate $\ge 10\,\mathrm{Hz}$ bei $640\times480$ (Richtwert; Modellabhängig)
- [ ] Events funktionieren:
  - [ ] `/vision/stop_request` wird in definierter Testsituation `true`
  - [ ] `/vision/slowdown_factor` reagiert (z. B. 1.0 → 0.4)
- [ ] Degradation ist definiert und getestet:
  - [ ] Wenn Kamera oder Detector ausfällt: innerhalb $\le 1\,\mathrm{s}$ wird Fallback aktiv (siehe Safety)
- [ ] Servo (optional):
  - [ ] `/pan/command` bewegt MG90S reproduzierbar (ohne Brownouts/Resets)

---

## 3) Datenverträge (Minimum, müssen stabil sein)

### 3.1 Kamera

| Topic | Richtung | Typ | Vertrag |
|---|---:|---|---|
| `/camera/image_raw` | Pub | `sensor_msgs/msg/Image` | `header.stamp` gesetzt, `frame_id=camera_link` |
| `/camera/camera_info` | Pub | `sensor_msgs/msg/CameraInfo` | Intrinsics/Distortion passend zu `image_raw` |

### 3.2 Vision/AI (Hailo)

| Topic | Richtung | Typ | Vertrag |
|---|---:|---|---|
| `/vision/detections` | Pub | `vision_msgs/msg/Detection2DArray` | bbox + Klasse + confidence |
| `/vision/stop_request` | Pub | `std_msgs/msg/Bool` | `true` = Stop anfordern |
| `/vision/slowdown_factor` | Pub | `std_msgs/msg/Float32` | Bereich $[0,1]$ |

### 3.3 Pan-Servo (optional, MG90S)

| Topic | Richtung | Typ | Vertrag |
|---|---:|---|---|
| `/pan/command` | Sub | `std_msgs/msg/Float32` | Winkel in $\mathrm{rad}$ (z. B. $[-1.2, +1.2]$) |
| `/pan/state` | Pub | `std_msgs/msg/Float32` | Ist-/Sollwinkel in $\mathrm{rad}$ |

Servo-PWM Rahmenwerte:

- $50\,\mathrm{Hz}$, Pulsbreite typ. $1{,}0\text{–}2{,}0\,\mathrm{ms}$ (oft $0{,}75\text{–}2{,}25\,\mathrm{ms}$ für vollen Weg). :contentReference[oaicite:5]{index=5}

---

## 4) Voraussetzungen (aus den Phasen)

### 4.1 Pflicht

- Phase 2: Pi 5 + ROS 2 Humble + Docker Tooling stabil
- Phase 3: LiDAR `/scan` stabil (primärer Kollisionsschutz)
- Phase 4: `base_link → camera_link` TF statisch vorhanden

### 4.2 Hardware-Voraussetzungen (diese Phase)

- **Raspberry Pi Global Shutter Camera (IMX296)** am CSI-Port. :contentReference[oaicite:6]{index=6}
- **Raspberry Pi AI Kit (Hailo-8L, 13 TOPS)** über M.2 HAT+ an **Pi 5 PCIe 2.0**. :contentReference[oaicite:7]{index=7}
- **MG90S Servo**:
  - Separate $5\,\mathrm{V}$ Versorgung empfohlen (Servo-Stall gemessen bis ~$700\,\mathrm{mA}$). :contentReference[oaicite:8]{index=8}
  - Gemeinsame Masse (GND) zwischen Servo-Netzteil und Pi.

---

## 5) Projektablage (Bringup im Repo)

```text
ros2_ws/src/
├─ amr_bringup/
│  ├─ launch/
│  │  ├─ bringup.launch.py
│  │  └─ vision_ai.launch.py
│  └─ config/
│     ├─ vision_camera.yaml
│     ├─ vision_hailo.yaml
│     └─ vision_events.yaml
├─ amr_vision_hailo/
│  ├─ launch/
│  │  └─ detector.launch.py
│  └─ config/
│     └─ models.yaml
└─ amr_pan_servo/
   ├─ launch/
   │  └─ pan.launch.py
   └─ config/
      └─ pan_limits.yaml
```

Ziel: **ein** Launch für Phase 7 (`vision_ai.launch.py`), der Kamera + Detector + Event-Node + optional Pan-Servo startet.

Hinweis: Hailo-Pipeline kann auf den offiziellen Pi-5 Examples/HailoRT aufsetzen (Referenz). ([GitHub][1])

---

## 9) Standard-Smoke-Tests (ohne RViz2)

### 9.1 Kamera (OS-Level)

```bash
rpicam-hello --list-cameras
rpicam-hello
```

(Bookworm-Tools; basieren auf libcamera). ([Raspberry Pi][2])

### 9.2 ROS Topics

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo -n 1 /camera/camera_info

ros2 topic hz /vision/detections
ros2 topic echo /vision/stop_request
ros2 topic echo /vision/slowdown_factor
```

### 9.3 Servo (optional)

```bash
ros2 topic pub /pan/command std_msgs/msg/Float32 "{data: 0.0}"
ros2 topic pub /pan/command std_msgs/msg/Float32 "{data: 0.8}"
ros2 topic pub /pan/command std_msgs/msg/Float32 "{data: -0.8}"
```

---

## 10) Typische Fehlerbilder (kurz, diagnostisch)

- **Kamera nicht sichtbar**: CSI/Kabel falsch, falscher Port, `rpicam-apps` fehlt, Kamera nicht erkannt (prüfe `--list-cameras`). ([Raspberry Pi][2])
- **Bildrate bricht ein**: Auflösung/Format zu hoch → zunächst $640\times480$ testen, QoS „Best Effort“ für Image.
- **Detektionen leer**: falsches Modell/Label-Mapping, confidence zu hoch, Input-Farbraum falsch.
- **Inferenz läuft auf CPU**: Hailo nicht initialisiert → HailoRT/Device prüfen; Beispiel-Referenz nutzen. ([GitHub][1])
- **Servo verursacht Resets**: Servo an Pi-$5,\mathrm{V}$ → Brownout. Separate Versorgung + GND gemeinsam. Stall bis ~$700,\mathrm{mA}$. ([ProtoSupplies][3])
- **Servo Weg zu klein**: Pulsbreite erweitern (z. B. $0{,}75\text{–}2{,}25,\mathrm{ms}$), mechanische Limits beachten. ([verical.com][4])

---

## 11) Sicherheitsanforderung (nicht verhandelbar)

- Vision ersetzt keine Safety-Sensorik: LiDAR + Not-Aus bleiben primär.
- Fallback bei Vision-Ausfall ist **projektweit festgelegt** (eine Option wählen und dokumentieren):

  - **A (Default für Feldbetrieb):** Vision-Ausfall → `slowdown_factor=1.0`, `stop_request=false` (Vision wird ignoriert)
  - **B (Konservativ/Testbetrieb):** Vision-Ausfall → `stop_request=true` (AMR bleibt stehen)
- Servo: Mechanische Limits + sichere Montage. Keine Pan-Bewegung darf Kabel/CSI-Strang belasten.
