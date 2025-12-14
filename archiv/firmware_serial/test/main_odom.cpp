/**
 * @file main.cpp
 * @brief AMR Serial-Bridge Firmware mit Odometrie (Phase 2)
 * @version 0.4.0-odom
 * @date 2025-12-12
 *
 * @standard REP-103 (SI-Einheiten), REP-105 (Frames)
 * @hardware Seeed Studio XIAO ESP32-S3, Cytron MDD3A, JGA25-370
 *
 * SERIAL-PROTOKOLL:
 *   Host → ESP32:  V:<m/s>,W:<rad/s>\n     (Geschwindigkeitsbefehl)
 *   Host → ESP32:  RESET_ODOM\n            (Odometrie zurücksetzen)
 *   ESP32 → Host:  OK:<v>,<w>\n            (Bestätigung)
 *   ESP32 → Host:  ODOM:<l>,<r>,<x>,<y>,<th>\n  (Odometrie-Daten)
 *   ESP32 → Host:  FAILSAFE:TIMEOUT\n      (Sicherheitsstopp)
 *
 * CHANGELOG v0.4.0:
 *   - Encoder-ISR implementiert
 *   - Odometrie-Berechnung (Differentialkinematik)
 *   - ODOM-Nachricht im Serial-Protokoll
 *   - Kalibrierte Encoder-Werte (374.3/373.6 Ticks/Rev)
 */

#include "config.h"
#include <Arduino.h>

// ==========================================================================
// GLOBALE VARIABLEN
// ==========================================================================

// --- Encoder (volatile für ISR-Zugriff) ---
volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

// --- Odometrie-Zustand ---
float odom_x = 0.0f;     // Position X [m]
float odom_y = 0.0f;     // Position Y [m]
float odom_theta = 0.0f; // Orientierung [rad]

long prev_ticks_left = 0; // Vorherige Encoder-Werte
long prev_ticks_right = 0;

// --- Motor-Steuerung ---
float target_v = 0.0f; // Ziel-Lineargeschwindigkeit [m/s]
float target_w = 0.0f; // Ziel-Winkelgeschwindigkeit [rad/s]

// --- Timing ---
unsigned long last_cmd_time = 0;
unsigned long last_odom_time = 0;
unsigned long last_loop_time = 0;

// --- Status ---
bool failsafe_active = false;

// ==========================================================================
// INTERRUPT SERVICE ROUTINES
// ==========================================================================

void IRAM_ATTR isr_encoder_left() { encoder_ticks_left++; }

void IRAM_ATTR isr_encoder_right() { encoder_ticks_right++; }

// ==========================================================================
// MOTOR HAL (Hardware Abstraction Layer)
// ==========================================================================

/**
 * @brief Initialisiert PWM-Kanäle für Motoren
 */
void hal_motor_init() {
    // PWM-Kanäle konfigurieren (ESP32 Arduino 2.x API)
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);

    // Pins an Kanäle binden
    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);

    // Initialer Zustand: Stopp
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

/**
 * @brief Setzt Motor-Geschwindigkeit mit Deadzone-Kompensation
 * @param ch_a PWM-Kanal A (Vorwärts)
 * @param ch_b PWM-Kanal B (Rückwärts)
 * @param speed Geschwindigkeit [-1.0, 1.0]
 */
void hal_motor_set(uint8_t ch_a, uint8_t ch_b, float speed) {
    // Clamp auf [-1, 1]
    speed = constrain(speed, -1.0f, 1.0f);

    // Unter 5% Totzone ignorieren
    if (fabs(speed) < 0.05f) {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, 0);
        return;
    }

    // Deadzone-Kompensation: Skaliere auf [PWM_DEADZONE, PWM_MAX]
    int pwm =
        PWM_DEADZONE + (int)(fabs(speed) * (MOTOR_PWM_MAX - PWM_DEADZONE));
    pwm = constrain(pwm, 0, MOTOR_PWM_MAX);

    if (speed > 0) {
        ledcWrite(ch_a, pwm);
        ledcWrite(ch_b, 0);
    } else {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, pwm);
    }
}

/**
 * @brief Stoppt beide Motoren sofort
 */
void hal_motor_stop() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

// ==========================================================================
// ENCODER HAL
// ==========================================================================

/**
 * @brief Initialisiert Encoder-Pins und Interrupts
 */
void hal_encoder_init() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isr_encoder_left,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isr_encoder_right,
                    RISING);
}

/**
 * @brief Liest Encoder-Ticks thread-sicher
 * @param left Zeiger auf linke Ticks
 * @param right Zeiger auf rechte Ticks
 */
void hal_encoder_read(long *left, long *right) {
    noInterrupts();
    *left = encoder_ticks_left;
    *right = encoder_ticks_right;
    interrupts();
}

/**
 * @brief Setzt Encoder-Ticks zurück
 */
void hal_encoder_reset() {
    noInterrupts();
    encoder_ticks_left = 0;
    encoder_ticks_right = 0;
    interrupts();
    prev_ticks_left = 0;
    prev_ticks_right = 0;
}

// ==========================================================================
// KINEMATIK
// ==========================================================================

/**
 * @brief Berechnet Rad-Geschwindigkeiten aus v und w (Differentialkinematik)
 * @param v Lineargeschwindigkeit [m/s]
 * @param w Winkelgeschwindigkeit [rad/s]
 * @param v_left Ausgabe: linke Radgeschwindigkeit [m/s]
 * @param v_right Ausgabe: rechte Radgeschwindigkeit [m/s]
 */
void kinematics_inverse(float v, float w, float *v_left, float *v_right) {
    // Differentialkinematik: v_l = v - w*L/2, v_r = v + w*L/2
    *v_left = v - (w * WHEEL_BASE / 2.0f);
    *v_right = v + (w * WHEEL_BASE / 2.0f);
}

/**
 * @brief Aktualisiert Odometrie aus Encoder-Deltas
 */
void odometry_update() {
    // Aktuelle Encoder-Werte lesen
    long ticks_left, ticks_right;
    hal_encoder_read(&ticks_left, &ticks_right);

    // Delta berechnen
    long delta_left = ticks_left - prev_ticks_left;
    long delta_right = ticks_right - prev_ticks_right;

    // Speichern für nächsten Zyklus
    prev_ticks_left = ticks_left;
    prev_ticks_right = ticks_right;

    // Sanity Check: Sprünge ignorieren
    if (abs(delta_left) > MAX_TICK_DELTA || abs(delta_right) > MAX_TICK_DELTA) {
        return;
    }

    // Strecken pro Rad berechnen (mit kalibrierten Werten)
    float d_left = delta_left * METERS_PER_TICK_LEFT;
    float d_right = delta_right * METERS_PER_TICK_RIGHT;

    // Differentialkinematik (Vorwärtskinematik)
    float d_center = (d_left + d_right) / 2.0f;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    // Pose aktualisieren (mit Mittelpunkt-Approximation)
    odom_x += d_center * cos(odom_theta + d_theta / 2.0f);
    odom_y += d_center * sin(odom_theta + d_theta / 2.0f);
    odom_theta += d_theta;

    // Theta normalisieren auf [-π, π]
    while (odom_theta > PI)
        odom_theta -= 2.0f * PI;
    while (odom_theta < -PI)
        odom_theta += 2.0f * PI;
}

/**
 * @brief Setzt Odometrie zurück
 */
void odometry_reset() {
    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_theta = 0.0f;
    hal_encoder_reset();
}

// ==========================================================================
// SERIAL-PROTOKOLL
// ==========================================================================

/**
 * @brief Verarbeitet eingehende Serial-Befehle
 */
void serial_process() {
    static String buffer = "";

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (buffer.length() > 0) {
                // Befehl parsen
                buffer.trim();

                // V:x.xx,W:y.yy Format
                if (buffer.startsWith("V:")) {
                    int comma_pos = buffer.indexOf(',');
                    if (comma_pos > 0 && buffer.indexOf("W:") > comma_pos) {
                        float v = buffer.substring(2, comma_pos).toFloat();
                        float w = buffer.substring(comma_pos + 3).toFloat();

                        // Limits prüfen
                        if (fabs(v) <= MAX_LINEAR_SPEED &&
                            fabs(w) <= MAX_ANGULAR_SPEED) {
                            target_v = v;
                            target_w = w;
                            last_cmd_time = millis();
                            failsafe_active = false;
                            Serial.printf("OK:%.2f,%.2f\n", v, w);
                        } else {
                            Serial.println("ERR:CMD_OUT_OF_RANGE");
                        }
                    } else {
                        Serial.println("ERR:PARSE_FAILED");
                    }
                }
                // RESET_ODOM Befehl
                else if (buffer == "RESET_ODOM") {
                    odometry_reset();
                    Serial.println("OK:ODOM_RESET");
                }
                // Unbekannter Befehl
                else {
                    Serial.println("ERR:UNKNOWN_CMD");
                }

                buffer = "";
            }
        } else {
            buffer += c;
            // Buffer-Overflow verhindern
            if (buffer.length() > 64) {
                buffer = "";
            }
        }
    }
}

/**
 * @brief Sendet Odometrie-Daten über Serial
 */
void serial_publish_odom() {
    long ticks_left, ticks_right;
    hal_encoder_read(&ticks_left, &ticks_right);

    // Format: ODOM:<left>,<right>,<x>,<y>,<theta>
    Serial.printf("ODOM:%ld,%ld,%.4f,%.4f,%.4f\n", ticks_left, ticks_right,
                  odom_x, odom_y, odom_theta);
}

// ==========================================================================
// MOTOR-STEUERUNG
// ==========================================================================

/**
 * @brief Aktualisiert Motor-Ausgänge basierend auf target_v und target_w
 */
void motor_update() {
    // Radgeschwindigkeiten berechnen
    float v_left, v_right;
    kinematics_inverse(target_v, target_w, &v_left, &v_right);

    // Normalisieren auf [-1, 1] basierend auf MAX_LINEAR_SPEED
    float speed_left = v_left / MAX_LINEAR_SPEED;
    float speed_right = v_right / MAX_LINEAR_SPEED;

    // Motoren ansteuern
    hal_motor_set(PWM_CH_LEFT_A, PWM_CH_LEFT_B, speed_left);
    hal_motor_set(PWM_CH_RIGHT_A, PWM_CH_RIGHT_B, speed_right);
}

/**
 * @brief Prüft Failsafe-Timeout
 */
void failsafe_check() {
    if (millis() - last_cmd_time > FAILSAFE_TIMEOUT_MS) {
        if (!failsafe_active) {
            target_v = 0.0f;
            target_w = 0.0f;
            hal_motor_stop();
            failsafe_active = true;
            Serial.println("FAILSAFE:TIMEOUT");
        }
    }
}

// ==========================================================================
// SETUP & LOOP
// ==========================================================================

void setup() {
    Serial.begin(115200);

    // Warten auf Serial (USB-CDC)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {
        delay(10);
    }

    // Hardware initialisieren
    hal_motor_init();
    hal_encoder_init();

    // LED für Statusanzeige (optional)
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_BITS);
    ledcAttachPin(PIN_LED_MOSFET, LED_PWM_CHANNEL);

    // Timing initialisieren
    last_cmd_time = millis();
    last_odom_time = millis();
    last_loop_time = millis();

    Serial.println("AMR-ESP32 v0.4.0-odom ready");
    Serial.println(
        "Protocol: V:<m/s>,W:<rad/s> | RESET_ODOM | ODOM:<l>,<r>,<x>,<y>,<th>");
}

void loop() {
    unsigned long now = millis();

    // Hauptschleife mit fester Rate (100 Hz)
    if (now - last_loop_time >= LOOP_PERIOD_MS) {
        last_loop_time = now;

        // 1. Befehle verarbeiten
        serial_process();

        // 2. Odometrie aktualisieren
        odometry_update();

        // 3. Failsafe prüfen
        failsafe_check();

        // 4. Motoren ansteuern
        motor_update();
    }

    // Odometrie publizieren (50 Hz)
    if (now - last_odom_time >= ODOM_PERIOD_MS) {
        last_odom_time = now;
        serial_publish_odom();
    }
}
