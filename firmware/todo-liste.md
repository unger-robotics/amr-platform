# ✅ Master-Checkliste: Phase 3 Validierung (Dual-Core & micro-ROS)

**Status:** 🚧 In Bearbeitung
**Ziel:** Nachweis, dass die neue Architektur (v3.0.0) die Fahrleistungen der alten Serial-Bridge (v0.5.0) erreicht oder übertrifft.

---

## 1. Low-Level Architektur-Test (ESP32-S3)

*Prüfung der Firmware-Stabilität und Echtzeit-Fähigkeit.*

* [ ] **Dual-Core Boot Check**
  * *Aktion:* ESP32 neu starten (Reset).
  * *Erwartung:* LED blinkt schnell (Suche Agent), dann dauerhaft AN (Verbunden). Kein "Boot-Loop" oder Absturz beim Verbindungsaufbau.
* [ ] **Heartbeat & Failsafe (Safety)**
  * *Aktion:* Verbindung trennen (Agent stoppen oder USB ziehen) während Motoren laufen (aufgebockt).
  * *Erwartung:* Motoren stoppen nach exakt **1000ms** (`FAILSAFE_TIMEOUT_MS`).
  * *Grund:* Validierung der Safety-Logik in `controlTask` auf Core 0.
* [ ] **Frequenz-Stabilität**
  * *Aktion:* `ros2 topic hz /odom_raw` auf dem Pi ausführen.
  * *Erwartung:* Stabile **20 Hz** (+/- 1 Hz).
  * *Grund:* Bestätigt, dass der `loop()` auf Core 1 nicht durch Core 0 blockiert wird und die Bandbreiten-Begrenzung greift.

## 2. PID & Regelung (Migrationstest)

*Validierung, ob die PID-Werte der alten Firmware in der neuen Umgebung funktionieren.*

* [ ] **Deadzone-Check**
  * *Aktion:* Langsamste Geschwindigkeit senden: `ros2 topic pub /cmd_vel ... linear: x: 0.05`.
  * *Erwartung:* Räder drehen sich gerade so. Kein "Singen" ohne Bewegung (PWM < Deadzone).
  * *Grund:* Die `hal_motor_write` Funktion muss die `PWM_DEADZONE` (35) korrekt addieren.
* [ ] **Geradeauslauf (1m Test)**
  * *Aktion:* `cmd_vel` mit `x: 0.2, z: 0.0` für 5 Sekunden senden.
  * *Erwartung:* Roboter fährt geradeaus, Drift < 2cm.
  * *Analyse:* Wenn er driftet, arbeiten die PID-Regler auf Core 0 (`pid_left`, `pid_right`) nicht synchron. Prüfen ob `vTaskDelayUntil` korrektes Timing liefert.
* [ ] **Sprungantwort (Reaktionszeit)**
  * *Aktion:* Plötzlicher Stopp von Volllast.
  * *Erwartung:* Sofortiger Stillstand, kein langes Nachlaufen.
  * *Grund:* Der D-Anteil ($K_d = 0.01$) muss das Bremsen unterstützen.

## 3. Odometrie-Validierung (Datenfluss)

*Vom Encoder-Tick bis zur ROS-Nachricht.*

* [ ] **Encoder-Richtung**
  * *Aktion:* Roboter manuell vorwärts schieben.
  * *Erwartung:* `encoder_ticks` müssen positiv zählen (Debugging evtl. temporär via Serial Print oder Blick auf `/odom_raw` X-Wert).
* [ ] **Bridge-Node Funktion (`odom_converter.py`)**
  * *Aktion:* `ros2 topic echo /odom`
  * *Erwartung:* Nachrichtentyp `nav_msgs/Odometry`. Werte für `pose.pose.position.x` sind nicht null.
  * *Wichtig:* `frame_id: odom` und `child_frame_id: base_link` müssen gesetzt sein.
* [ ] **Drehung (Winkel-Check)**
  * *Aktion:* Roboter um exakt 360° (eine Umdrehung) am Boden drehen.
  * *Erwartung:* Der Wert `theta` in `/odom_raw` (oder konvertiert in Quaternion in `/odom`) sollte wieder bei ca. 0 (bzw. $2\pi$) ankommen.
  * *Fehlerquelle:* Wenn der Wert stark abweicht (z.B. nur 180° in Software), stimmt `WHEEL_BASE` in `config.h` nicht.

## 4. RViz Integrationstest (End-to-End)

*Visualisierung und TF-Tree.*

* [ ] **TF-Baum Konsistenz**
  * *Aktion:* `ros2 run tf2_tools view_frames`.
  * *Erwartung:* Ein Baum: `odom` -> `base_link`. Zeitstempel müssen aktuell sein ("Average rate" ca. 20 Hz).
* [ ] **Live-Visualisierung**
  * *Aktion:* RViz2 starten, Fixed Frame = `odom`.
  * *Test:* Teleop starten und Kreise fahren.
  * *Erwartung:* Die Bewegung in RViz ist flüssig (kein Springen/Ruckeln). Der Roboter kehrt in RViz an den Startpunkt zurück, wenn er es in echt tut.
  * *Indikator für Erfolg:* Das "Dual-Core" Design zahlt sich aus – die Odometrie wird auch bei hoher CPU-Last oder WLAN-Jitter weiterintegriert.

---

### 🚨 Troubleshooting Guide für Phase 3

| Symptom | Check | Lösung |
| :--- | :--- | :--- |
| **Agent verbindet nicht** | Baudrate / Kabel | Kabel prüfen (Datenleitungen?). Baud 115200 in `platformio.ini` und Agent-Startbefehl gleich? |
| **Odom driftet massiv** | Encoder Ticks | `METERS_PER_TICK` in `config.h` neu berechnen. Prüfen ob Encoderscheibe rutscht. |
| **Keine Daten in RViz** | QoS Policy | Prüfen ob `odom_converter.py` wirklich `ReliabilityPolicy.BEST_EFFORT` nutzt. |
| **TF Error in RViz** | Zeitstempel | Systemzeit von Pi und PC synchron? (NTP prüfen). |

Sobald diese Liste abgehakt ist, ist die **Phase 3 (micro-ROS & Odometrie)** offiziell abgeschlossen und das Fundament für Phase 4 (SLAM) steht.
