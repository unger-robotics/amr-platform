// =============================================================================
// micro-ROS Motor-Control Firmware für AMR
// Version: 2.0.0 | Phase 3.2
// =============================================================================
//
// Topics:
//   Publisher:  /esp32/heartbeat     (std_msgs/Int32)  - Watchdog
//   Subscriber: /cmd_vel             (geometry_msgs/Twist) - Motorsteuerung
//   Subscriber: /esp32/led_cmd       (std_msgs/Bool)   - LED-Steuerung
//
// Hardware: Seeed Studio XIAO ESP32-S3 + Cytron MDD3A
// =============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

// =============================================================================
// Pin-Definitionen (aus config.h, hier nochmal für Klarheit)
// =============================================================================
#define LED_PIN 21 // Onboard LED (active LOW)

// Motor Pins sind in config.h definiert:
// PIN_MOTOR_LEFT_A  (D0), PIN_MOTOR_LEFT_B  (D1)
// PIN_MOTOR_RIGHT_A (D2), PIN_MOTOR_RIGHT_B (D3)

// =============================================================================
// micro-ROS Objekte
// =============================================================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Publisher
rcl_publisher_t pub_heartbeat;
std_msgs__msg__Int32 msg_heartbeat;

// Subscribers
rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_led_cmd;
geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Bool msg_led_cmd;

// Executor
rclc_executor_t executor;

// =============================================================================
// Zustandsvariablen
// =============================================================================
volatile float target_linear = 0.0f;  // m/s
volatile float target_angular = 0.0f; // rad/s
unsigned long last_cmd_time = 0;      // Für Failsafe
int32_t heartbeat_counter = 0;

// =============================================================================
// Motor-Steuerung
// =============================================================================

void setupMotors() {
    // PWM Kanäle konfigurieren (ESP32 Arduino 2.x API)
    ledcSetup(PWM_CH_LEFT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);

    // Pins an Kanäle anhängen
    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);

    // Motoren stoppen
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

void stopMotors() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

// Deadzone-Kompensation: PWM-Wert anpassen
uint8_t applyDeadzone(float speed_normalized) {
    if (fabs(speed_normalized) < 0.01f) {
        return 0;
    }
    // Skaliere auf PWM-Bereich oberhalb der Deadzone
    uint8_t pwm = (uint8_t)(PWM_DEADZONE + fabs(speed_normalized) *
                                               (MOTOR_PWM_MAX - PWM_DEADZONE));
    return min(pwm, (uint8_t)MOTOR_PWM_MAX);
}

// Motor mit Richtung und Geschwindigkeit ansteuern
// speed: -1.0 bis +1.0 (normalisiert)
void setMotorLeft(float speed) {
    uint8_t pwm = applyDeadzone(speed);

    if (speed > 0.01f) {
        // Vorwärts
        ledcWrite(PWM_CH_LEFT_A, pwm);
        ledcWrite(PWM_CH_LEFT_B, 0);
    } else if (speed < -0.01f) {
        // Rückwärts
        ledcWrite(PWM_CH_LEFT_A, 0);
        ledcWrite(PWM_CH_LEFT_B, pwm);
    } else {
        // Stopp (Coast)
        ledcWrite(PWM_CH_LEFT_A, 0);
        ledcWrite(PWM_CH_LEFT_B, 0);
    }
}

void setMotorRight(float speed) {
    uint8_t pwm = applyDeadzone(speed);

    if (speed > 0.01f) {
        // Vorwärts
        ledcWrite(PWM_CH_RIGHT_A, pwm);
        ledcWrite(PWM_CH_RIGHT_B, 0);
    } else if (speed < -0.01f) {
        // Rückwärts
        ledcWrite(PWM_CH_RIGHT_A, 0);
        ledcWrite(PWM_CH_RIGHT_B, pwm);
    } else {
        // Stopp (Coast)
        ledcWrite(PWM_CH_RIGHT_A, 0);
        ledcWrite(PWM_CH_RIGHT_B, 0);
    }
}

// =============================================================================
// Differential Drive Kinematik
// =============================================================================
// Konvertiert (v, ω) zu (v_left, v_right)
// v: lineare Geschwindigkeit [m/s]
// ω: Winkelgeschwindigkeit [rad/s]

void updateMotors() {
    // Geschwindigkeiten begrenzen
    float v = constrain(target_linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    float w = constrain(target_angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    // Differential Drive Kinematik
    // v_left  = v - (ω * L/2)
    // v_right = v + (ω * L/2)
    float v_left = v - (w * WHEEL_BASE / 2.0f);
    float v_right = v + (w * WHEEL_BASE / 2.0f);

    // Normalisieren auf -1.0 bis +1.0
    float v_left_norm = v_left / MAX_LINEAR_SPEED;
    float v_right_norm = v_right / MAX_LINEAR_SPEED;

    // Clamp
    v_left_norm = constrain(v_left_norm, -1.0f, 1.0f);
    v_right_norm = constrain(v_right_norm, -1.0f, 1.0f);

    // Motoren ansteuern
    setMotorLeft(v_left_norm);
    setMotorRight(v_right_norm);
}

// =============================================================================
// Callbacks
// =============================================================================

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    target_linear = msg->linear.x;
    target_angular = msg->angular.z;
    last_cmd_time = millis();
}

void led_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    digitalWrite(PIN_LED_MOSFET, msg->data ? HIGH : LOW);
}

// =============================================================================
// Failsafe
// =============================================================================

void checkFailsafe() {
    if (millis() - last_cmd_time > FAILSAFE_TIMEOUT_MS) {
        // Timeout - Motoren stoppen
        target_linear = 0.0f;
        target_angular = 0.0f;
        stopMotors();
    }
}

// =============================================================================
// LED Helper
// =============================================================================

void setStatusLED(bool on) {
    // XIAO LED ist active LOW
    digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    // Status LED
    pinMode(LED_PIN, OUTPUT);
    setStatusLED(false);

    // MOSFET LED
    pinMode(PIN_LED_MOSFET, OUTPUT);
    digitalWrite(PIN_LED_MOSFET, LOW);

    // Motoren initialisieren
    setupMotors();

    // USB Serial für micro-ROS
    Serial.begin(115200);
    delay(2000);

    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    // Warte auf Agent (blockierend, mit Blinken)
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        setStatusLED((millis() / 250) % 2);
        delay(100);
    }

    // Agent gefunden
    setStatusLED(true);

    // Support initialisieren
    rclc_support_init(&support, 0, NULL, &allocator);

    // Node erstellen
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Publisher: Heartbeat
    rclc_publisher_init_best_effort(
        &pub_heartbeat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32/heartbeat");

    // Subscriber: cmd_vel (Best Effort für Performance)
    rclc_subscription_init_best_effort(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    // Subscriber: LED
    rclc_subscription_init_default(
        &sub_led_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "esp32/led_cmd");

    // Executor mit 2 Subscriptions
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
                                   &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_led_cmd, &msg_led_cmd,
                                   &led_callback, ON_NEW_DATA);

    // Initiale Zeit setzen
    last_cmd_time = millis();
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
    // micro-ROS Executor
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Failsafe prüfen
    checkFailsafe();

    // Motoren aktualisieren
    updateMotors();

    // Heartbeat (1 Hz)
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 1000) {
        msg_heartbeat.data = heartbeat_counter++;
        (void)rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);
        last_heartbeat = millis();
    }

    // Kurze Pause für Stabilität
    delay(10);
}
