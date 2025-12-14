/**
 * @file config.h
 * @brief Zentrale Konfiguration für AMR Slave-Node (ESP32-S3)
 * @version 1.3.0
 * @date 2025-12-12
 *
 * @standard REP-103 (SI-Einheiten), REP-105 (Frames), Safety-First
 * @hardware Seeed Studio XIAO ESP32-S3, Cytron MDD3A, JGA25-370
 *
 * CHANGELOG v1.3.0:
 *   - PID-Parameter für Geschwindigkeitsregelung
 *   - Tuning-Startwerte basierend auf Systemcharakteristik
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==========================================================================
// 1. HARDWARE ABSTRACTION LAYER (HAL)
// ==========================================================================

// --- Antriebsstrang (Cytron MDD3A - DUAL PWM MODE) ---
#define PIN_MOTOR_LEFT_A D0  // MDD3A M1A (Vorwärts-PWM)
#define PIN_MOTOR_LEFT_B D1  // MDD3A M1B (Rückwärts-PWM)
#define PIN_MOTOR_RIGHT_A D2 // MDD3A M2A (Vorwärts-PWM)
#define PIN_MOTOR_RIGHT_B D3 // MDD3A M2B (Rückwärts-PWM)

// PWM-Kanäle (ESP32 LEDC)
#define PWM_CH_LEFT_A 0
#define PWM_CH_LEFT_B 1
#define PWM_CH_RIGHT_A 2
#define PWM_CH_RIGHT_B 3

// --- Odometrie (Hall-Encoder JGA25-370) ---
#define PIN_ENC_LEFT_A D6  // Interrupt-fähig
#define PIN_ENC_RIGHT_A D7 // Interrupt-fähig

// --- Peripherie & Status ---
#define PIN_LED_MOSFET D10 // IRLZ24N Low-Side Switch

// --- I2C Bus (MPU6050 / Future Use) ---
#define PIN_I2C_SDA D4
#define PIN_I2C_SCL D5
#define IMU_I2C_ADDR 0x68

// --- Servos (Pan/Tilt - Optional) ---
#define PIN_SERVO_PAN D8
#define PIN_SERVO_TILT D9

// ==========================================================================
// 2. KINEMATISCHE PARAMETER (SI-Einheiten / REP-103)
// ==========================================================================

#define WHEEL_DIAMETER 0.065f // [m] Raddurchmesser
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0f)
#define WHEEL_BASE 0.178f // [m] Spurbreite
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.14159265359f)

// ==========================================================================
// 2.1 ENCODER-KALIBRIERUNG (2025-12-12)
// ==========================================================================
// Methode: 10-Umdrehungen-Test
// Ergebnis: Links 3743 Ticks, Rechts 3736 Ticks

#define TICKS_PER_REV_LEFT 374.3f  // Kalibriert
#define TICKS_PER_REV_RIGHT 373.6f // Kalibriert
#define TICKS_PER_REV ((TICKS_PER_REV_LEFT + TICKS_PER_REV_RIGHT) / 2.0f)

#define METERS_PER_TICK_LEFT (WHEEL_CIRCUMFERENCE / TICKS_PER_REV_LEFT)
#define METERS_PER_TICK_RIGHT (WHEEL_CIRCUMFERENCE / TICKS_PER_REV_RIGHT)

// ==========================================================================
// 3. REGELUNGSTECHNIK
// ==========================================================================

// --- PWM Konfiguration ---
#define MOTOR_PWM_FREQ 20000 // 20 kHz (unhörbar)
#define MOTOR_PWM_BITS 8     // 8-bit Auflösung (0-255)
#define MOTOR_PWM_MAX 255

// --- Motor Deadzone ---
#define PWM_DEADZONE 35 // PWM unter dem Motor nicht anläuft

// --- LED PWM ---
#define LED_PWM_FREQ 5000
#define LED_PWM_BITS 8
#define LED_PWM_CHANNEL 4

// ==========================================================================
// 3.1 PID-REGLER PARAMETER
// ==========================================================================
//
// Tuning-Methode: Manuell (inkrementell) am 2025-12-12
//
// Systemcharakteristik (aus Bodentest):
//   - Soll-Geschwindigkeit: 0.2 m/s
//   - Open-Loop Drift: 14 cm/m (rechts 3.7% schneller)
//   - Open-Loop Überschuss: 16% (1.158 m statt 1.0 m)
//
// Tuning-Ergebnis:
//   - Distanzfehler: 1.6% (0.984 m bei 1.0 m Soll)
//   - Drift: 0.5 cm (vorher 14 cm)
//   - Encoder-Synchronisation: 1802/1802 Ticks (perfekt)
//
// Tuning-Prozess:
//   1. Kp schrittweise erhöht (2.0 → 13.0) bis Soll-Geschwindigkeit erreicht
//   2. Ki erhöht (0.5 → 5.0) bis stationärer Fehler eliminiert
//   3. Kd beibehalten (0.01) - kein Überschwingen beobachtet

#define PID_KP 1.0f // 0.5f // Drift nach links, Proportional - Hauptkorrektur
#define PID_KI 0.0f // 0.0f  // Integral - Drift + stationärer Fehler
#define PID_KD 0.0f // 0.01f // Derivative - Dämpfung

// --- Geschwindigkeitslimits ---
#define MAX_LINEAR_SPEED 0.5f  // [m/s]
#define MAX_ANGULAR_SPEED 2.0f // [rad/s]

// ==========================================================================
// 4. SAFETY STANDARDS
// ==========================================================================

// Erhöht auf 1000ms für stabilere DDS-Verbindung (verhindert Ruckeln)
#define FAILSAFE_TIMEOUT_MS 4000

#define ENABLE_TASK_WDT true
#define TASK_WDT_TIMEOUT_S 5

// Maximaler Encoder-Sprung pro Zyklus (Sanity Check)
#define MAX_TICK_DELTA 50

// ==========================================================================
// 5. TIMING & LOOP RATES
// ==========================================================================

#define LOOP_RATE_HZ 100                     // Haupt-Regelschleife (Core 0)
#define LOOP_PERIOD_MS (1000 / LOOP_RATE_HZ) // 10ms

// Reduziert auf 20 Hz, um Serial-Bandbreite für Befehle freizuhalten
#define ODOM_PUBLISH_HZ 20
#define ODOM_PERIOD_MS (1000 / ODOM_PUBLISH_HZ) // 50ms

// ==========================================================================
// 6. DEBUG & DIAGNOSTICS
// ==========================================================================

#define DEBUG_SERIAL false
#define DEBUG_BAUD 115200
#define ENABLE_DIAGNOSTICS false

#endif // CONFIG_H
