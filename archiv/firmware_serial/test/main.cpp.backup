/**
 * @file main.cpp
 * @brief AMR Serial-Bridge Firmware für ESP32-S3
 * @version 0.3.0-serial
 * @date 2025-12-12
 *
 * @standard REP-103 (SI-Einheiten), REP-105 (Frames), Safety-First
 *
 * Protokoll:
 *   Empfang: "V:<linear_m/s>,W:<angular_rad/s>\n"
 *   Beispiel: "V:0.20,W:0.50\n"
 *
 * Failsafe: Motoren stoppen nach 500ms ohne gültigen Befehl
 */

#include "config.h"
#include <Arduino.h>

// ==========================================================================
// GLOBALE VARIABLEN
// ==========================================================================

float cmd_linear = 0.0f;  // Ziel: lineare Geschwindigkeit [m/s]
float cmd_angular = 0.0f; // Ziel: Winkelgeschwindigkeit [rad/s]
unsigned long last_cmd_time = 0;
unsigned long last_loop_time = 0;
bool failsafe_active = false;

// Serial Buffer
char serial_buffer[64];
uint8_t buffer_idx = 0;

// ==========================================================================
// HARDWARE ABSTRACTION
// ==========================================================================

void hal_motor_init() {
    // PWM Kanäle konfigurieren (ESP32 Arduino 2.x API)
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_BITS);

    // Pins an Kanäle binden
    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);
    ledcAttachPin(PIN_LED_MOSFET, LED_PWM_CHANNEL);
}

/**
 * @brief Setzt Motor-PWM mit Deadzone-Kompensation
 * @param ch_a PWM-Kanal A (Vorwärts)
 * @param ch_b PWM-Kanal B (Rückwärts)
 * @param speed Normiert: -1.0 bis +1.0
 *
 * Deadzone-Kompensation: Skaliert [5%, 100%] auf [PWM_DEADZONE, 255]
 * Damit laufen auch kleine Geschwindigkeiten zuverlässig an.
 */
void hal_motor_set(uint8_t ch_a, uint8_t ch_b, float speed) {
    speed = constrain(speed, -1.0f, 1.0f);

    // Unter 5% ignorieren (echte Totzone)
    if (abs(speed) < 0.05f) {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, 0);
        return;
    }

    // Deadzone-Kompensation: Skaliere auf [PWM_DEADZONE, PWM_MAX]
    int pwm = PWM_DEADZONE + (int)(abs(speed) * (MOTOR_PWM_MAX - PWM_DEADZONE));

    if (speed >= 0) {
        ledcWrite(ch_a, pwm);
        ledcWrite(ch_b, 0);
    } else {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, pwm);
    }
}

void motors_stop() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

// ==========================================================================
// DIFFERENTIAL DRIVE KINEMATIK (REP-103 konform)
// ==========================================================================

/**
 * @brief Berechnet Radgeschwindigkeiten aus cmd_vel
 *
 * Differentialkinematik:
 *   v_left  = v - (ω × L/2)
 *   v_right = v + (ω × L/2)
 *
 * wobei L = Spurbreite (WHEEL_BASE)
 */
void apply_cmd_vel(float v_linear, float v_angular) {
    // Radgeschwindigkeiten berechnen [m/s]
    float v_left = v_linear - (v_angular * WHEEL_BASE / 2.0f);
    float v_right = v_linear + (v_angular * WHEEL_BASE / 2.0f);

    // Maximale Radgeschwindigkeit aus Kinematik
    // v_max_wheel = v_max + (ω_max × L/2)
    const float MAX_WHEEL_SPEED =
        MAX_LINEAR_SPEED + (MAX_ANGULAR_SPEED * WHEEL_BASE / 2.0f);

    // Normieren auf [-1, 1] für PWM
    float speed_left = v_left / MAX_WHEEL_SPEED;
    float speed_right = v_right / MAX_WHEEL_SPEED;

    // Clipping mit Skalierung (behält Kurvenverhältnis bei)
    if (abs(speed_left) > 1.0f || abs(speed_right) > 1.0f) {
        float scale = max(abs(speed_left), abs(speed_right));
        speed_left /= scale;
        speed_right /= scale;
    }

    // Motoren ansteuern
    hal_motor_set(PWM_CH_LEFT_A, PWM_CH_LEFT_B, speed_left);
    hal_motor_set(PWM_CH_RIGHT_A, PWM_CH_RIGHT_B, speed_right);
}

// ==========================================================================
// SERIAL PROTOKOLL PARSER
// ==========================================================================

/**
 * @brief Parst eingehende Befehle
 * @return true wenn gültiger Befehl erkannt
 *
 * Format: "V:<float>,W:<float>\n"
 * Beispiel: "V:0.20,W:-0.50\n"
 */
bool parse_command(const char *cmd) {
    float v = 0.0f, w = 0.0f;

    // Format: V:xxx,W:xxx
    if (cmd[0] != 'V' || cmd[1] != ':')
        return false;

    char *comma = strchr(cmd, ',');
    if (!comma)
        return false;

    // Linear velocity parsen
    v = atof(cmd + 2);

    // Angular velocity parsen
    char *w_start = strchr(comma, 'W');
    if (!w_start || w_start[1] != ':')
        return false;
    w = atof(w_start + 2);

    // Plausibilitätsprüfung (150% Toleranz für Rampen)
    if (abs(v) > MAX_LINEAR_SPEED * 1.5f || abs(w) > MAX_ANGULAR_SPEED * 1.5f) {
        Serial.println("ERR:CMD_OUT_OF_RANGE");
        return false;
    }

    // Werte übernehmen (auf Limits begrenzt)
    cmd_linear = constrain(v, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    cmd_angular = constrain(w, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    last_cmd_time = millis();

    return true;
}

void process_serial() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (buffer_idx > 0) {
                serial_buffer[buffer_idx] = '\0';

                if (parse_command(serial_buffer)) {
                    Serial.print("OK:");
                    Serial.print(cmd_linear, 2);
                    Serial.print(",");
                    Serial.println(cmd_angular, 2);
                    failsafe_active = false;
                }

                buffer_idx = 0;
            }
        } else if (buffer_idx < sizeof(serial_buffer) - 1) {
            serial_buffer[buffer_idx++] = c;
        }
    }
}

// ==========================================================================
// LED FEEDBACK
// ==========================================================================

void led_update() {
    static unsigned long last_led = 0;
    static uint8_t brightness = 0;
    static int8_t direction = 5;

    if (millis() - last_led < 20)
        return;
    last_led = millis();

    if (failsafe_active) {
        // Schnelles Blinken bei Failsafe
        brightness = (millis() / 100) % 2 ? 255 : 0;
    } else if (cmd_linear != 0.0f || cmd_angular != 0.0f) {
        // Dauerlicht bei Bewegung
        brightness = 200;
    } else {
        // Sanftes Atmen im Idle
        brightness += direction;
        if (brightness >= 100 || brightness <= 5)
            direction = -direction;
    }

    ledcWrite(LED_PWM_CHANNEL, brightness);
}

// ==========================================================================
// SETUP & LOOP
// ==========================================================================

void setup() {
    Serial.begin(DEBUG_BAUD);
    delay(1000); // USB-CDC stabilisieren

    hal_motor_init();
    motors_stop();

    Serial.println("AMR Serial-Bridge v0.3.0");
    Serial.println("Format: V:<m/s>,W:<rad/s>");
    Serial.println("READY");

    last_cmd_time = millis();
    last_loop_time = millis();
}

void loop() {
    // Non-Blocking Loop Rate Control (100 Hz)
    if (millis() - last_loop_time < LOOP_PERIOD_MS) {
        return;
    }
    last_loop_time = millis();

    // 1. Serial-Befehle verarbeiten
    process_serial();

    // 2. Failsafe prüfen (Safety-First)
    if (millis() - last_cmd_time > FAILSAFE_TIMEOUT_MS) {
        if (!failsafe_active) {
            motors_stop();
            cmd_linear = 0.0f;
            cmd_angular = 0.0f;
            failsafe_active = true;
            Serial.println("FAILSAFE:TIMEOUT");
        }
    }

    // 3. Motoren aktualisieren (nur wenn kein Failsafe)
    if (!failsafe_active) {
        apply_cmd_vel(cmd_linear, cmd_angular);
    }

    // 4. LED Status
    led_update();
}
