# ESP32-S3 XIAO Standard-Testprogramm

> **Version:** 1.0.1 | **Stand:** 2025-12-12

Minimales Testprogramm zur Hardware-Validierung des ESP32-S3 XIAO.

---

## Funktion

- LED-Strip an D10 über MOSFET IRLZ24N
- Sanftes Breathing (Auf- und Abdimmen)
- ~3 Sekunden pro Zyklus
- Verwendet `config.h` für Hardware-Definitionen
- **Non-Blocking Code** (kein `delay()` im Loop)

---

## Hardware

| Komponente | Pin | Beschreibung |
|------------|-----|--------------|
| LED-Strip | D10 | Über MOSFET IRLZ24N (Low-Side Switch) |

---

## Flashen

```bash
cd esp32_test
pio run --target upload
pio device monitor
```

---

## Erwartete Ausgabe

```
================================
  ESP32-S3 XIAO Testprogramm
================================
LED Pin: D10 (GPIO 3)
PWM Kanal: 4
PWM Frequenz: 5000 Hz
Effekt: Breathing

System bereit.
```

---

## Dateien

```
esp32_test/
├── platformio.ini      # PlatformIO Konfiguration
├── include/
│   └── config.h        # Hardware-Parameter
├── src/
│   └── main.cpp        # Hauptprogramm
└── README.md           # Diese Datei
```

---

## Parameter anpassen

In `src/main.cpp`:

| Parameter | Standard | Beschreibung |
|-----------|----------|--------------|
| `BREATH_STEP` | 5 | Helligkeitsschritte (größer = schneller) |
| `BREATH_DELAY` | 30 ms | Pause zwischen Schritten |
| `BREATH_MAX` | 255 | Maximale Helligkeit |

---

*<https://github.com/unger-robotics/amr-platform>*
