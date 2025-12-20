# Phase 1 – Befehlsreferenz

## Nach Pi Reboot

```bash
cd ~/amr-platform/docker
docker compose up -d
sleep 5
docker compose logs microros_agent --tail 5
```

**Erwartung:** `running... | fd: 3`

---

## Nach ESP32 Reboot (USB umgesteckt)

```bash
cd ~/amr-platform/docker
docker compose restart microros_agent
sleep 5
docker compose logs microros_agent --tail 5
```

**Erwartung:** `running... | fd: 3`

---

## Notfall-Stop (falls Räder drehen)

```bash
docker compose exec amr_dev bash -c "source /opt/ros/humble/setup.bash && \
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'"
```

---

## Smoke-Tests

### 1. In Container gehen

```bash
docker compose exec amr_dev bash
source /opt/ros/humble/setup.bash
```

### 2. Topics prüfen

```bash
ros2 topic list
```

**Erwartung:**

```
/cmd_vel
/esp32/heartbeat
/esp32/led_cmd
/odom_raw
/parameter_events
/rosout
```

### 3. Heartbeat prüfen

```bash
ros2 topic echo /esp32/heartbeat
```

**Erwartung:** Counter incrementiert ~1×/s (Ctrl+C zum Beenden)

### 4. Odometrie prüfen

```bash
ros2 topic echo /odom_raw --once
```

**Erwartung:** x, y, theta Werte

### 5. Frequenzen messen

```bash
ros2 topic hz /esp32/heartbeat
```

```bash
ros2 topic hz /odom_raw
```

**Erwartung:** Heartbeat ~1 Hz, Odom ~3-6 Hz

---

## Motor-Tests (⚠️ Räder aufbocken!)

### Vorwärts

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}" -r 10
```

### Rückwärts

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.15}, angular: {z: 0.0}}" -r 10
```

### Drehen links

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

### Drehen rechts

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.5}}" -r 10
```

### Manueller Stop

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**Hinweis:** Nach Ctrl+C stoppt der Failsafe die Motoren automatisch nach 2 Sekunden.

---

## Failsafe-Test

1. Motor-Befehl senden (Räder drehen)
2. `Ctrl+C` drücken
3. 2 Sekunden warten
4. **Erwartung:** Motoren stoppen automatisch

---

## Odom nach Fahrt prüfen

```bash
ros2 topic echo /odom_raw --once
```

**Erwartung:** x > 0 nach Vorwärtsfahrt, theta ändert sich nach Drehung

---

## Checkliste Phase 1

| Test | Erwartung | Status |
|------|-----------|--------|
| Agent verbindet | `fd: 3` | ☐ |
| Topics vorhanden | 6 Topics | ☐ |
| Heartbeat | ~1 Hz | ☐ |
| Odom publiziert | x, y, theta | ☐ |
| Vorwärts | Räder drehen vorwärts | ☐ |
| Rückwärts | Räder drehen rückwärts | ☐ |
| Drehen links | Roboter dreht links | ☐ |
| Drehen rechts | Roboter dreht rechts | ☐ |
| Failsafe | Stop nach 2s | ☐ |

---

## Troubleshooting

| Problem | Lösung |
|---------|--------|
| `Serial port not found` | USB-Kabel prüfen, `ls /dev/ttyACM*` |
| Topics fehlen | `docker compose restart microros_agent` |
| Räder reagieren nicht | Feedforward-Gain prüfen (2.0) |
| Falsches Verhalten | USB ziehen, Firmware prüfen |
