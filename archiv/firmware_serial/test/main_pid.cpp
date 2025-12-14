/**
 * @file main.cpp
 * @brief AMR Firmware mit PID-Geschwindigkeitsregelung (Phase 2)
 * @version 0.5.0-pid
 * @date 2025-12-12
 *
 * @standard REP-103 (SI-Einheiten), REP-105 (Frames)
 * @hardware Seeed Studio XIAO ESP32-S3, Cytron MDD3A, JGA25-370
 *
 * FEATURES:
 *   - Closed-Loop Geschwindigkeitsregelung (PID pro Rad)
 *   - Encoder-basierte Ist-Geschwindigkeit
 *   - Odometrie (x, y, theta)
 *   - Failsafe Timeout
 *
 * SERIAL-PROTOKOLL:
 *   Host → ESP32:  V:<m/s>,W:<rad/s>\n
 *   Host → ESP32:  RESET_ODOM\n
 *   Host → ESP32:  PID:<Kp>,<Ki>,<Kd>\n        (Live-Tuning)
 *   ESP32 → Host:  ODOM:<l>,<r>,<x>,<y>,<th>\n
 *   ESP32 → Host:  VEL:<v_l>,<v_r>,<pwm_l>,<pwm_r>\n  (Debug)
 */

#include <Arduino.h>
#include "config.h"

// ==========================================================================
// PID-REGLER KLASSE
// ==========================================================================

class PIDController {
public:
    float Kp, Ki, Kd;
    
    PIDController(float kp, float ki, float kd) 
        : Kp(kp), Ki(ki), Kd(kd), integral(0), prev_error(0) {}
    
    /**
     * @brief Berechnet PID-Ausgabe
     * @param setpoint Soll-Wert [m/s]
     * @param measurement Ist-Wert [m/s]
     * @param dt Zeitschritt [s]
     * @return Stellgröße [-1.0, 1.0]
     */
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        // Integral mit Anti-Windup
        integral += error * dt;
        integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        
        // Derivative (mit Filter gegen Rauschen)
        float derivative = 0;
        if (dt > 0.001f) {
            derivative = (error - prev_error) / dt;
        }
        prev_error = error;
        
        // PID-Summe
        float output = Kp * error + Ki * integral + Kd * derivative;
        
        return constrain(output, -1.0f, 1.0f);
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        reset();  // Reset bei Gain-Änderung
    }

private:
    static constexpr float INTEGRAL_MAX = 0.5f;  // Anti-Windup Limit
    float integral;
    float prev_error;
};

// ==========================================================================
// GLOBALE VARIABLEN
// ==========================================================================

// --- Encoder (volatile für ISR) ---
volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

// --- Geschwindigkeitsmessung ---
long prev_ticks_left = 0;
long prev_ticks_right = 0;
float velocity_left = 0.0f;   // Ist-Geschwindigkeit [m/s]
float velocity_right = 0.0f;

// --- Odometrie ---
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// --- Sollwerte ---
float target_v = 0.0f;        // Linear [m/s]
float target_w = 0.0f;        // Angular [rad/s]
float target_v_left = 0.0f;   // Rad-Sollgeschwindigkeit [m/s]
float target_v_right = 0.0f;

// --- PID-Regler (pro Rad) ---
PIDController pid_left(PID_KP, PID_KI, PID_KD);
PIDController pid_right(PID_KP, PID_KI, PID_KD);

// --- PWM-Ausgabe ---
float pwm_left = 0.0f;
float pwm_right = 0.0f;

// --- Timing ---
unsigned long last_cmd_time = 0;
unsigned long last_odom_time = 0;
unsigned long last_loop_time = 0;
unsigned long last_vel_time = 0;

// --- Status ---
bool failsafe_active = false;
bool debug_velocity = false;  // VEL-Nachrichten aktivieren

// ==========================================================================
// INTERRUPT SERVICE ROUTINES
// ==========================================================================

void IRAM_ATTR isr_encoder_left() {
    encoder_ticks_left++;
}

void IRAM_ATTR isr_encoder_right() {
    encoder_ticks_right++;
}

// ==========================================================================
// HARDWARE ABSTRACTION LAYER
// ==========================================================================

void hal_motor_init() {
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);

    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);

    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

void hal_encoder_init() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), 
                    isr_encoder_left, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), 
                    isr_encoder_right, RISING);
}

void hal_encoder_read(long* left, long* right) {
    noInterrupts();
    *left = encoder_ticks_left;
    *right = encoder_ticks_right;
    interrupts();
}

void hal_encoder_reset() {
    noInterrupts();
    encoder_ticks_left = 0;
    encoder_ticks_right = 0;
    interrupts();
    prev_ticks_left = 0;
    prev_ticks_right = 0;
}

/**
 * @brief Setzt Motor-PWM (Dual-PWM für MDD3A)
 * @param ch_a PWM-Kanal A (Vorwärts)
 * @param ch_b PWM-Kanal B (Rückwärts)
 * @param speed Normierte Geschwindigkeit [-1.0, 1.0]
 */
void hal_motor_set_pwm(uint8_t ch_a, uint8_t ch_b, float speed) {
    speed = constrain(speed, -1.0f, 1.0f);

    // Totzone: Unter 3% ignorieren (für PID wichtig, kleiner als Open-Loop)
    if (fabs(speed) < 0.03f) {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, 0);
        return;
    }

    // PWM berechnen mit Deadzone-Kompensation
    int pwm = PWM_DEADZONE + (int)(fabs(speed) * (MOTOR_PWM_MAX - PWM_DEADZONE));
    pwm = constrain(pwm, 0, MOTOR_PWM_MAX);

    if (speed > 0) {
        ledcWrite(ch_a, pwm);
        ledcWrite(ch_b, 0);
    } else {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, pwm);
    }
}

void hal_motor_stop() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
    
    pwm_left = 0;
    pwm_right = 0;
}

// ==========================================================================
// GESCHWINDIGKEITSBERECHNUNG
// ==========================================================================

/**
 * @brief Berechnet Ist-Geschwindigkeiten aus Encoder-Delta
 * @param dt Zeitschritt [s]
 */
void velocity_update(float dt) {
    long ticks_left, ticks_right;
    hal_encoder_read(&ticks_left, &ticks_right);

    long delta_left = ticks_left - prev_ticks_left;
    long delta_right = ticks_right - prev_ticks_right;

    prev_ticks_left = ticks_left;
    prev_ticks_right = ticks_right;

    // Sanity Check
    if (abs(delta_left) > MAX_TICK_DELTA || abs(delta_right) > MAX_TICK_DELTA) {
        return;
    }

    // Geschwindigkeit [m/s] = Strecke / Zeit
    if (dt > 0.001f) {
        float dist_left = delta_left * METERS_PER_TICK_LEFT;
        float dist_right = delta_right * METERS_PER_TICK_RIGHT;
        
        // Tiefpass-Filter für Rauschunterdrückung
        // v_filtered = alpha * v_new + (1-alpha) * v_old
        const float alpha = 0.3f;
        velocity_left = alpha * (dist_left / dt) + (1.0f - alpha) * velocity_left;
        velocity_right = alpha * (dist_right / dt) + (1.0f - alpha) * velocity_right;
    }
}

// ==========================================================================
// ODOMETRIE
// ==========================================================================

void odometry_update(float dt) {
    // Strecken aus Geschwindigkeiten (genauer als Tick-Delta bei PID)
    float d_left = velocity_left * dt;
    float d_right = velocity_right * dt;

    float d_center = (d_left + d_right) / 2.0f;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    odom_x += d_center * cos(odom_theta + d_theta / 2.0f);
    odom_y += d_center * sin(odom_theta + d_theta / 2.0f);
    odom_theta += d_theta;

    // Normalisieren auf [-π, π]
    while (odom_theta > PI) odom_theta -= 2.0f * PI;
    while (odom_theta < -PI) odom_theta += 2.0f * PI;
}

void odometry_reset() {
    odom_x = 0.0f;
    odom_y = 0.0f;
    odom_theta = 0.0f;
    velocity_left = 0.0f;
    velocity_right = 0.0f;
    hal_encoder_reset();
    pid_left.reset();
    pid_right.reset();
}

// ==========================================================================
// KINEMATIK
// ==========================================================================

void kinematics_inverse(float v, float w, float* v_left, float* v_right) {
    *v_left  = v - (w * WHEEL_BASE / 2.0f);
    *v_right = v + (w * WHEEL_BASE / 2.0f);
}

// ==========================================================================
// MOTOR-REGELUNG
// ==========================================================================

/**
 * @brief Aktualisiert PID-Regler und setzt PWM
 * @param dt Zeitschritt [s]
 */
void motor_control_update(float dt) {
    // Sollgeschwindigkeiten für Räder berechnen
    kinematics_inverse(target_v, target_w, &target_v_left, &target_v_right);

    // PID-Regelung
    if (fabs(target_v_left) < 0.01f && fabs(target_v_right) < 0.01f) {
        // Stopp: PID umgehen, direkt auf 0
        hal_motor_stop();
        pid_left.reset();
        pid_right.reset();
        return;
    }

    // PID-Ausgabe berechnen
    pwm_left = pid_left.compute(target_v_left, velocity_left, dt);
    pwm_right = pid_right.compute(target_v_right, velocity_right, dt);

    // Motoren ansteuern
    hal_motor_set_pwm(PWM_CH_LEFT_A, PWM_CH_LEFT_B, pwm_left);
    hal_motor_set_pwm(PWM_CH_RIGHT_A, PWM_CH_RIGHT_B, pwm_right);
}

// ==========================================================================
// SERIAL-PROTOKOLL
// ==========================================================================

void serial_process() {
    static String buffer = "";

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (buffer.length() > 0) {
                buffer.trim();

                // V:x.xx,W:y.yy
                if (buffer.startsWith("V:")) {
                    int comma_pos = buffer.indexOf(',');
                    if (comma_pos > 0 && buffer.indexOf("W:") > comma_pos) {
                        float v = buffer.substring(2, comma_pos).toFloat();
                        float w = buffer.substring(comma_pos + 3).toFloat();

                        if (fabs(v) <= MAX_LINEAR_SPEED && fabs(w) <= MAX_ANGULAR_SPEED) {
                            target_v = v;
                            target_w = w;
                            last_cmd_time = millis();
                            failsafe_active = false;
                            Serial.printf("OK:%.2f,%.2f\n", v, w);
                        } else {
                            Serial.println("ERR:CMD_OUT_OF_RANGE");
                        }
                    }
                }
                // RESET_ODOM
                else if (buffer == "RESET_ODOM") {
                    odometry_reset();
                    Serial.println("OK:ODOM_RESET");
                }
                // PID:<Kp>,<Ki>,<Kd> - Live-Tuning
                else if (buffer.startsWith("PID:")) {
                    int c1 = buffer.indexOf(',');
                    int c2 = buffer.indexOf(',', c1 + 1);
                    if (c1 > 0 && c2 > c1) {
                        float kp = buffer.substring(4, c1).toFloat();
                        float ki = buffer.substring(c1 + 1, c2).toFloat();
                        float kd = buffer.substring(c2 + 1).toFloat();
                        
                        pid_left.setGains(kp, ki, kd);
                        pid_right.setGains(kp, ki, kd);
                        Serial.printf("OK:PID=%.2f,%.2f,%.2f\n", kp, ki, kd);
                    }
                }
                // DEBUG:ON / DEBUG:OFF - Velocity-Nachrichten
                else if (buffer == "DEBUG:ON") {
                    debug_velocity = true;
                    Serial.println("OK:DEBUG_ON");
                }
                else if (buffer == "DEBUG:OFF") {
                    debug_velocity = false;
                    Serial.println("OK:DEBUG_OFF");
                }
                else {
                    Serial.println("ERR:UNKNOWN_CMD");
                }

                buffer = "";
            }
        } else {
            buffer += c;
            if (buffer.length() > 64) buffer = "";
        }
    }
}

void serial_publish_odom() {
    long ticks_left, ticks_right;
    hal_encoder_read(&ticks_left, &ticks_right);

    Serial.printf("ODOM:%ld,%ld,%.4f,%.4f,%.4f\n",
                  ticks_left, ticks_right,
                  odom_x, odom_y, odom_theta);
}

void serial_publish_velocity() {
    if (debug_velocity) {
        Serial.printf("VEL:%.3f,%.3f,%.3f,%.3f\n",
                      velocity_left, velocity_right,
                      pwm_left, pwm_right);
    }
}

// ==========================================================================
// FAILSAFE
// ==========================================================================

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

    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {
        delay(10);
    }

    hal_motor_init();
    hal_encoder_init();

    // LED für Status
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_BITS);
    ledcAttachPin(PIN_LED_MOSFET, LED_PWM_CHANNEL);

    last_cmd_time = millis();
    last_odom_time = millis();
    last_loop_time = millis();
    last_vel_time = millis();

    Serial.println("AMR-ESP32 v0.5.0-pid ready");
    Serial.printf("PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", PID_KP, PID_KI, PID_KD);
    Serial.println("Commands: V:v,W:w | RESET_ODOM | PID:Kp,Ki,Kd | DEBUG:ON/OFF");
}

void loop() {
    unsigned long now = millis();

    // Hauptschleife (100 Hz)
    if (now - last_loop_time >= LOOP_PERIOD_MS) {
        float dt = (now - last_loop_time) / 1000.0f;
        last_loop_time = now;

        // 1. Befehle verarbeiten
        serial_process();

        // 2. Geschwindigkeit messen
        velocity_update(dt);

        // 3. Odometrie aktualisieren
        odometry_update(dt);

        // 4. Failsafe prüfen
        failsafe_check();

        // 5. PID-Regelung + Motoren
        motor_control_update(dt);
    }

    // Odometrie publizieren (50 Hz)
    if (now - last_odom_time >= ODOM_PERIOD_MS) {
        last_odom_time = now;
        serial_publish_odom();
    }

    // Velocity Debug (10 Hz)
    if (now - last_vel_time >= 100) {
        last_vel_time = now;
        serial_publish_velocity();
    }
}
