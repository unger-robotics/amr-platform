/**
 * @file main.cpp
 * @brief Firmware für den ESP32-S3 basierten AMR Low-Level Controller.
 *
 * Diese Firmware implementiert eine Dual-Core Architektur zur strikten
 * Trennung von Echtzeit-Regelung (Motorsteuerung) und High-Level Kommunikation
 * (micro-ROS). Sie folgt den Standards REP-103 (Einheiten) und REP-105 (Frames)
 * gemäß Industriestandards-AMR.md.
 *
 * @version 3.1.0
 * @date 2025-12-20
 * @author Jan Unger
 *
 * @section architecture Architektur
 * - **Core 0 (Pro CPU):** Harte Echtzeit-Tasks (100 Hz PID Loop, Odometrie,
 * Safety).
 * - **Core 1 (App CPU):** Kommunikation (micro-ROS Agent, DDS Serialisierung).
 * - **Shared Memory:** Synchronisation via FreeRTOS Mutex.
 *
 * @section safety Sicherheitsfunktionen
 * - Dead Man's Switch (Heartbeat Timeout -> Motor Stop)
 * - Thread-Safe Data Exchange (Mutex Protection)
 * - Deterministisches Timing durch vTaskDelayUntil (Jitter-Free PID)
 *
 * CHANGELOG v3.1.0:
 *   - PID-Regler aktiviert (war Open-Loop Test-Code)
 *   - Startwerte: Kp=1.0, Ki=0.0, Kd=0.0 (aus config.h)
 */

// =============================================================================
// Includes & Dependencies
// =============================================================================
#include <Arduino.h>
#include <Wire.h> // I2C Kommunikation (für spätere IMU Integration)
#include <micro_ros_platformio.h>

// ROS 2 Client Library (rclc)
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// ROS 2 Nachrichtentypen
#include <geometry_msgs/msg/pose2_d.h> // Optimierte Odometrie (x, y, theta)
#include <geometry_msgs/msg/twist.h>   // Geschwindigkeitsbefehle
#include <std_msgs/msg/bool.h>         // LED/Status Steuerung
#include <std_msgs/msg/int32.h>        // Heartbeat

#include "config.h" // Zentrale Hardware- und Physik-Konfiguration

// =============================================================================
// FreeRTOS: Shared Memory & Synchronization
// =============================================================================
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

/**
 * @struct SharedData
 * @brief Thread-sicherer Datenspeicher für den Austausch zwischen Core 0 und
 * Core 1.
 *
 * Dient als "Briefkasten". Core 1 schreibt Befehle, Core 0 liest sie.
 * Core 0 schreibt Odometrie, Core 1 liest sie.
 * Zugriff muss durch `dataMutex` geschützt werden.
 */
struct SharedData {
    // --- Input (Geschrieben von Core 1 / Gelesen von Core 0) ---
    float target_lin_x;          ///< Soll-Linear-Geschwindigkeit [m/s]
    float target_ang_z;          ///< Soll-Winkel-Geschwindigkeit [rad/s]
    bool led_cmd_active;         ///< Status der LED/MOSFET (true = AN)
    unsigned long last_cmd_time; ///< Zeitstempel des letzten empfangenen
                                 ///< Befehls [ms] für Safety Timeout

    // --- Output (Geschrieben von Core 0 / Gelesen von Core 1) ---
    float odom_x;     ///< Aktuelle X-Position [m] (relativ zum Start)
    float odom_y;     ///< Aktuelle Y-Position [m]
    float odom_theta; ///< Aktuelle Orientierung [rad]
};

SharedData shared_data;      ///< Globale Instanz der Shared Data
SemaphoreHandle_t dataMutex; ///< Mutex zum Schutz vor Race Conditions

// =============================================================================
// PID Regler Klasse
// =============================================================================

/**
 * @class PIDController
 * @brief Implementiert einen diskreten PID-Regler mit Anti-Windup.
 *
 * Verwendet die Standardform: Output = Kp*e + Ki*∫e + Kd*de/dt.
 * Entwickelt in Phase 2 für Drift-Kompensation.
 */
class PIDController {
  public:
    float Kp; ///< Proportional-Beiwert (Reaktionsschnelle)
    float Ki; ///< Integral-Beiwert (Stationäre Genauigkeit)
    float Kd; ///< Derivative-Beiwert (Dämpfung)

    /**
     * @brief Konstruktor für den PID Regler.
     * @param kp Proportional-Gain
     * @param ki Integral-Gain
     * @param kd Derivative-Gain
     */
    PIDController(float kp, float ki, float kd)
        : Kp(kp), Ki(ki), Kd(kd), integral(0), prev_error(0) {}

    /**
     * @brief Berechnet den Stellwert (PWM-Faktor).
     *
     * @param setpoint Sollwert [m/s]
     * @param measurement Istwert [m/s]
     * @param dt Zeitdifferenz seit letztem Aufruf [s]
     * @return float Stellwert im Bereich [-1.0, 1.0]
     */
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        // Integral-Anteil berechnen & begrenzen (Anti-Windup)
        integral += error * dt;
        integral = constrain(integral, -0.5f, 0.5f);

        // Derivative-Anteil berechnen (verhindert Division durch Null)
        float derivative = (dt > 0.0001f) ? (error - prev_error) / dt : 0;
        prev_error = error;

        float output = Kp * error + Ki * integral + Kd * derivative;
        return constrain(output, -1.0f, 1.0f); // Normierung auf PWM-Bereich
    }

    /**
     * @brief Setzt den internen Status (Integral, Fehler) zurück.
     * Wichtig beim Stoppen oder Richtungswechsel (Failsafe Reset).
     */
    void reset() {
        integral = 0;
        prev_error = 0;
    }

  private:
    float integral;   ///< Akkumulierter Fehler (I-Anteil)
    float prev_error; ///< Fehler des letzten Zyklus (für D-Anteil)
};

// PID Instanzen für linken und rechten Motor (Parameter aus config.h)
PIDController pid_left(PID_KP, PID_KI, PID_KD);
PIDController pid_right(PID_KP, PID_KI, PID_KD);

// =============================================================================
// Globale Hardware Variablen (Interrupt Service Routinen)
// =============================================================================

// Volatile ist zwingend, da Zugriff aus ISR und Main-Loop erfolgt
volatile long encoder_ticks_left = 0;  ///< Zähler linkes Rad (Inkremental)
volatile long encoder_ticks_right = 0; ///< Zähler rechtes Rad (Inkremental)

/**
 * @brief ISR für linken Encoder. Wird bei steigender Flanke ausgelöst.
 * Attribut IRAM_ATTR zwingt Code in den RAM für schnellere Ausführung
 * (Latenzminimierung).
 */
void IRAM_ATTR isr_encoder_left() { encoder_ticks_left++; }

/**
 * @brief ISR für rechten Encoder.
 */
void IRAM_ATTR isr_encoder_right() { encoder_ticks_right++; }

// =============================================================================
// ROS Objekte (Core 1)
// =============================================================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Publisher
rcl_publisher_t
    pub_odom; ///< Publiziert Pose2D (x,y,theta) - Optimiert für Bandbreite
rcl_publisher_t pub_heartbeat; ///< Publiziert Lebenszeichen (Watchdog)
geometry_msgs__msg__Pose2D msg_odom;
std_msgs__msg__Int32 msg_heartbeat;

// Subscriber
rcl_subscription_t sub_cmd_vel; ///< Empfängt Twist Nachrichten (Steuerung)
rcl_subscription_t sub_led_cmd; ///< Empfängt LED Befehle (Status/Scheinwerfer)
geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Bool msg_led_cmd;

// =============================================================================
// Hardware Abstraction Layer (HAL)
// =============================================================================

/**
 * @brief Initialisiert alle Hardware-Pins und Peripheriegeräte.
 * Konfiguriert PWM, Encoder-Interrupts, I2C und Safety-Pins gemäß config.h.
 */
void hal_init_complete() {
    // 1. Motoren (PWM via LEDC) - Frequenz ca. 20kHz für Silent Operation
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);

    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);

    // 2. Encoder (Input Pullup + Interrupts) - Hall-Sensoren benötigen Pullups
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isr_encoder_left,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isr_encoder_right,
                    RISING);

    // 3. Safety & Status
    pinMode(PIN_LED_MOSFET, OUTPUT);
    digitalWrite(PIN_LED_MOSFET, LOW); // Default: AUS (Safety First)
    pinMode(21, OUTPUT);               // Onboard LED

    // 4. I2C Bus Init (für MPU6050 in Phase 3)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

    // 5. Servo Pins Init (Reserve für Phase 5 Camera Pan/Tilt)
    pinMode(PIN_SERVO_PAN, OUTPUT);
    pinMode(PIN_SERVO_TILT, OUTPUT);
}

/**
 * @brief Setzt die PWM-Werte für die Motoren basierend auf dem normalisierten
 * Input. Implementiert die Logik für den Cytron MDD3A Treiber (Dual PWM Mode).
 *
 * @param v_left Geschwindigkeit links [-1.0 ... 1.0]
 * @param v_right Geschwindigkeit rechts [-1.0 ... 1.0]
 */
void hal_motor_write(float v_left, float v_right) {
    // Lambda-Funktion zur Ansteuerung eines einzelnen Motors
    auto set_pwm = [](int ch_a, int ch_b, float speed) {
        if (fabs(speed) <
            0.05f) { // Software Deadzone verhindert "Singen" der Motoren
            ledcWrite(ch_a, 0);
            ledcWrite(ch_b, 0);
        } else {
            // Skalierung auf PWM-Bereich (Deadzone bis Max)
            int pwm = PWM_DEADZONE +
                      (int)(fabs(speed) * (MOTOR_PWM_MAX - PWM_DEADZONE));
            pwm = constrain(pwm, 0, MOTOR_PWM_MAX);

            if (speed > 0) { // Vorwärts
                ledcWrite(ch_a, pwm);
                ledcWrite(ch_b, 0);
            } else { // Rückwärts
                ledcWrite(ch_a, 0);
                ledcWrite(ch_b, pwm);
            }
        }
    };

    set_pwm(PWM_CH_LEFT_A, PWM_CH_LEFT_B, v_left);
    set_pwm(PWM_CH_RIGHT_A, PWM_CH_RIGHT_B, v_right);
}

// =============================================================================
// CORE 0: Real-Time Control Task
// =============================================================================

/**
 * @brief Echtzeit-Task für Regelung und Odometrie (läuft auf Core 0).
 *
 * Dieser Task läuft mit fester Frequenz (100 Hz) und höchster Priorität.
 * Er ist physikalisch entkoppelt von der ROS-Kommunikation auf Core 1.
 *
 * @param pvParameters FreeRTOS Task Parameter (nicht genutzt)
 */
void controlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency =
        pdMS_TO_TICKS(LOOP_PERIOD_MS); // 10ms aus config.h

    // Lokale Odometrie-Akkumulatoren (werden hier integriert)
    float local_x = 0;
    float local_y = 0;
    float local_theta = 0;

    // Encoder Status
    long prev_ticks_l = 0;
    long prev_ticks_r = 0;
    float v_enc_l = 0; // Gefilterte Geschwindigkeit
    float v_enc_r = 0;

    for (;;) {
        // --- 1. DATEN SICHER ABHOLEN (Mutex Lock) ---
        float target_v = 0;
        float target_w = 0;
        bool failsafe = false;
        bool led_on = false;

        if (xSemaphoreTake(dataMutex, (TickType_t)2) == pdTRUE) {
            target_v = shared_data.target_lin_x;
            target_w = shared_data.target_ang_z;
            led_on = shared_data.led_cmd_active;

            // Safety-Check: Heartbeat Timeout prüfen (gemäß Industriestandard)
            if (millis() - shared_data.last_cmd_time > FAILSAFE_TIMEOUT_MS) {
                failsafe = true;
            }
            xSemaphoreGive(dataMutex);
        }

        // --- 2. ODOMETRIE & PHYSIK BERECHNEN ---
        float dt =
            LOOP_PERIOD_MS / 1000.0f; // Festes dt = 0.01s (Deterministisch!)

        // Encoder atomar auslesen (Interrupts kurz sperren)
        noInterrupts();
        long curr_l = encoder_ticks_left;
        long curr_r = encoder_ticks_right;
        interrupts();

        long dn_l = curr_l - prev_ticks_l;
        long dn_r = curr_r - prev_ticks_r;
        prev_ticks_l = curr_l;
        prev_ticks_r = curr_r;

        // Plausibilitätsprüfung (Filterung von Glitches / Kontaktprellen)
        if (abs(dn_l) < MAX_TICK_DELTA && abs(dn_r) < MAX_TICK_DELTA) {
            // Umrechnung Ticks -> Meter
            float dist_l = dn_l * METERS_PER_TICK_LEFT;
            float dist_right = dn_r * METERS_PER_TICK_RIGHT;

            // 1. Rohe Geschwindigkeit berechnen (immer positiv bei
            // Single-Channel Encoder)
            float raw_v_l = dist_l / dt;
            float raw_v_r = dist_right / dt;

            // 2. RICHTUNGS-HEURISTIK (Der entscheidende Fix!)
            // Wenn wir rückwärts fahren WOLLEN (target < 0), machen wir die
            // Messung negativ. Wir greifen hier auf target_v zu, das wir oben
            // aus dem Mutex geholt haben. Da wir Differential Drive haben,
            // schauen wir auf die individuelle Rad-Anforderung.

            // Ziel-Geschwindigkeit für die Räder berechnen (aus target_v und
            // target_w)
            float check_set_v_l = target_v - (target_w * WHEEL_BASE / 2.0f);
            float check_set_v_r = target_v + (target_w * WHEEL_BASE / 2.0f);

            if (check_set_v_l < -0.01)
                raw_v_l = -fabs(raw_v_l);
            if (check_set_v_r < -0.01)
                raw_v_r = -fabs(raw_v_r);

            // 3. Low-Pass Filter anwenden
            v_enc_l = 0.7 * v_enc_l + 0.3 * raw_v_l;
            v_enc_r = 0.7 * v_enc_r + 0.3 * raw_v_r;

            // Odometrie Integration (Differential Drive Kinematik)
            float d_center = (dist_l + dist_right) / 2.0f;
            float d_theta = (dist_right - dist_l) / WHEEL_BASE;

            local_x += d_center * cos(local_theta + d_theta / 2.0f);
            local_y += d_center * sin(local_theta + d_theta / 2.0f);
            local_theta += d_theta;
        }

        // --- 3. PID & MOTOR CONTROL ---
        if (failsafe) {
            // Not-Halt bei Kommunikationsverlust
            target_v = 0;
            target_w = 0;
            pid_left.reset();
            pid_right.reset();
            digitalWrite(PIN_LED_MOSFET, LOW); // Aktoren stromlos schalten
        } else {
            digitalWrite(PIN_LED_MOSFET, led_on ? HIGH : LOW);
        }

        // Inverse Kinematik: Twist (v, w) -> Radgeschwindigkeiten (v_l, v_r)
        float set_v_l = target_v - (target_w * WHEEL_BASE / 2.0f);
        float set_v_r = target_v + (target_w * WHEEL_BASE / 2.0f);

        // PID Berechnung (Parameter aus config.h: Kp=1.0, Ki=0.0, Kd=0.0)
        float pwm_l = pid_left.compute(set_v_l, v_enc_l, dt);
        float pwm_r = pid_right.compute(set_v_r, v_enc_r, dt);

        // Hardware ansteuern
        hal_motor_write(pwm_l, pwm_r);

        // --- 4. ERGEBNISSE ZURÜCKSCHREIBEN (Mutex Lock) ---
        if (xSemaphoreTake(dataMutex, (TickType_t)2) == pdTRUE) {
            shared_data.odom_x = local_x;
            shared_data.odom_y = local_y;
            shared_data.odom_theta = local_theta;
            xSemaphoreGive(dataMutex);
        }

        // --- 5. WAIT UNTIL ---
        // Wartet exakt bis zum nächsten 10ms Slot (Verhindert Drift im Timing)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =============================================================================
// ROS Callbacks (Core 1)
// =============================================================================

/**
 * @brief Callback für /cmd_vel Nachrichten.
 * Schreibt Zielwerte in den Shared Memory.
 */
void cb_cmd_vel(const void *msgin) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;
    if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE) {
        shared_data.target_lin_x = msg->linear.x;
        shared_data.target_ang_z = msg->angular.z;
        shared_data.last_cmd_time =
            millis(); // Reset Watchdog Timer (Heartbeat)
        xSemaphoreGive(dataMutex);
    }
}

/**
 * @brief Callback für LED Steuerung.
 */
void cb_led(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    if (xSemaphoreTake(dataMutex, (TickType_t)5) == pdTRUE) {
        shared_data.led_cmd_active = msg->data;
        xSemaphoreGive(dataMutex);
    }
}

// =============================================================================
// SETUP (Initialisierung)
// =============================================================================

void setup() {
    // 1. Hardware Init
    hal_init_complete();
    Serial.begin(115200);

    // 2. Shared Memory & Mutex Init
    dataMutex = xSemaphoreCreateMutex();
    shared_data.last_cmd_time = millis();

    // 3. Start Control Task auf Core 0 (Priorität: Hoch)
    // Stackgröße: 4096 Byte, Prio: configMAX_PRIORITIES - 1
    xTaskCreatePinnedToCore(controlTask, "ControlLoop", 4096, NULL,
                            configMAX_PRIORITIES - 1, NULL, 0);

    // 4. micro-ROS Init (Warten auf Agent)
    delay(1000);
    set_microros_serial_transports(Serial);

    // Blockiert, bis Agent antwortet (Ping Loop - Visuelles Feedback via LED)
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        digitalWrite(21, !digitalRead(21)); // Blinken als Status
        delay(100);
    }
    digitalWrite(21, LOW); // Verbunden (LED AN - Active Low)

    // ROS Node & Executor Setup
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_dual_core", "", &support);

    // Publisher Init (Best Effort für Sensor-Daten für weniger Overhead)
    rclc_publisher_init_best_effort(
        &pub_odom, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "odom_raw");

    rclc_publisher_init_best_effort(
        &pub_heartbeat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32/heartbeat");

    // Subscriber Init (Reliable für Steuerbefehle, da Verlust kritisch)
    rclc_subscription_init_default(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    rclc_subscription_init_default(
        &sub_led_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "esp32/led_cmd");

    // Executor Konfiguration (Verarbeitet Callbacks)
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
                                   &cb_cmd_vel, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_led_cmd, &msg_led_cmd,
                                   &cb_led, ON_NEW_DATA);
}

// =============================================================================
// MAIN LOOP (Core 1: Communication Only)
// =============================================================================

void loop() {
    // 1. micro-ROS Kommunikation verarbeiten
    // Spinnt für kurze Zeit, um eingehende Pakete zu checken
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    // 2. Odometrie Publizieren (Rate limitiert auf 20 Hz zur
    // Bandbreitenschonung)
    static unsigned long last_pub = 0;
    if (millis() - last_pub > ODOM_PERIOD_MS) { // 50ms = 20Hz (aus config.h)
        last_pub = millis();

        // Daten sicher aus Shared Memory lesen
        if (xSemaphoreTake(dataMutex, (TickType_t)2) == pdTRUE) {
            msg_odom.x = shared_data.odom_x;
            msg_odom.y = shared_data.odom_y;
            msg_odom.theta = shared_data.odom_theta;
            xSemaphoreGive(dataMutex);

            rcl_publish(&pub_odom, &msg_odom, NULL);
        }
    }

    // 3. Heartbeat Publizieren (1 Hz Diagnose)
    static unsigned long last_beat = 0;
    if (millis() - last_beat > 1000) {
        last_beat = millis();
        msg_heartbeat.data++;
        rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);
    }
}
