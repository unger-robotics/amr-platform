// =============================================================================
// micro-ROS AMR Firmware mit PID-Regelung
// Version: 2.2.2 | Produktion
// =============================================================================
//
// Topics:
//   Publisher:  /odom/x            (std_msgs/Float32)    - Position X [m]
//   Publisher:  /odom/y            (std_msgs/Float32)    - Position Y [m]
//   Publisher:  /odom/theta        (std_msgs/Float32)    - Orientation [rad]
//   Publisher:  /esp32/heartbeat   (std_msgs/Int32)      - Watchdog
//   Subscriber: /cmd_vel           (geometry_msgs/Twist) - Motorsteuerung
//   Subscriber: /esp32/led_cmd     (std_msgs/Bool)       - LED-Steuerung
//
// Features:
//   - Closed-Loop PID-Geschwindigkeitsregelung (pro Rad)
//   - Encoder-basierte Ist-Geschwindigkeit
//   - Odometrie (x, y, theta)
//   - Failsafe Timeout (1s)
//   - Optimierte Serial-Bandbreite (20Hz Odom)
//
// Hardware: Seeed Studio XIAO ESP32-S3 + Cytron MDD3A + JGA25-370 Encoder
// =============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

// =============================================================================
// Pin-Definitionen
// =============================================================================
#define LED_PIN 21 // Onboard LED (active LOW)

// =============================================================================
// PID-Regler Klasse
// =============================================================================

class PIDController {
  public:
    float Kp, Ki, Kd;

    PIDController(float kp, float ki, float kd)
        : Kp(kp), Ki(ki), Kd(kd), integral(0), prev_error(0) {}

    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        // Integral mit Anti-Windup
        integral += error * dt;
        integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

        // Derivative
        float derivative = 0;
        if (dt > 0.001f) {
            derivative = (error - prev_error) / dt;
        }
        prev_error = error;

        float output = Kp * error + Ki * integral + Kd * derivative;
        return constrain(output, -1.0f, 1.0f);
    }

    void reset() {
        integral = 0;
        prev_error = 0;
    }

  private:
    static constexpr float INTEGRAL_MAX = 0.5f;
    float integral;
    float prev_error;
};

// =============================================================================
// micro-ROS Objekte
// =============================================================================
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Publisher (nur 4 Topics für stabile Serial)
rcl_publisher_t pub_heartbeat;
rcl_publisher_t pub_odom_x;
rcl_publisher_t pub_odom_y;
rcl_publisher_t pub_odom_theta;
std_msgs__msg__Int32 msg_heartbeat;
std_msgs__msg__Float32 msg_odom_x;
std_msgs__msg__Float32 msg_odom_y;
std_msgs__msg__Float32 msg_odom_theta;

// Subscribers
rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_led_cmd;
geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Bool msg_led_cmd;

// Executor
rclc_executor_t executor;

// =============================================================================
// Encoder (volatile für ISR)
// =============================================================================
volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;
long prev_ticks_left = 0;
long prev_ticks_right = 0;

// =============================================================================
// Geschwindigkeitsmessung
// =============================================================================
float velocity_left = 0.0f;
float velocity_right = 0.0f;

// =============================================================================
// Odometrie
// =============================================================================
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;

// =============================================================================
// Motorsteuerung & PID
// =============================================================================
volatile float target_linear = 0.0f;
volatile float target_angular = 0.0f;
float target_v_left = 0.0f;
float target_v_right = 0.0f;

PIDController pid_left(PID_KP, PID_KI, PID_KD);
PIDController pid_right(PID_KP, PID_KI, PID_KD);

// =============================================================================
// Timing
// =============================================================================
unsigned long last_cmd_time = 0;
unsigned long last_loop_time = 0;
unsigned long last_odom_time = 0;
int32_t heartbeat_counter = 0;

// =============================================================================
// ISR
// =============================================================================
void IRAM_ATTR isr_encoder_left() { encoder_ticks_left++; }
void IRAM_ATTR isr_encoder_right() { encoder_ticks_right++; }

// =============================================================================
// Hardware Setup
// =============================================================================

void hal_encoder_init() {
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), isr_encoder_left,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), isr_encoder_right,
                    RISING);
}

void hal_encoder_read(long *left, long *right) {
    noInterrupts();
    *left = encoder_ticks_left;
    *right = encoder_ticks_right;
    interrupts();
}

void setupMotors() {
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

void stopMotors() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

void hal_motor_set_pwm(uint8_t ch_a, uint8_t ch_b, float speed) {
    speed = constrain(speed, -1.0f, 1.0f);

    if (fabs(speed) < 0.03f) {
        ledcWrite(ch_a, 0);
        ledcWrite(ch_b, 0);
        return;
    }

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

// =============================================================================
// Geschwindigkeitsberechnung
// =============================================================================

void velocity_update(float dt) {
    long ticks_left, ticks_right;
    hal_encoder_read(&ticks_left, &ticks_right);

    long delta_left = ticks_left - prev_ticks_left;
    long delta_right = ticks_right - prev_ticks_right;

    prev_ticks_left = ticks_left;
    prev_ticks_right = ticks_right;

    if (abs(delta_left) > MAX_TICK_DELTA || abs(delta_right) > MAX_TICK_DELTA) {
        return;
    }

    if (dt > 0.001f) {
        float dist_left = delta_left * METERS_PER_TICK_LEFT;
        float dist_right = delta_right * METERS_PER_TICK_RIGHT;

        const float alpha = 0.3f;
        velocity_left =
            alpha * (dist_left / dt) + (1.0f - alpha) * velocity_left;
        velocity_right =
            alpha * (dist_right / dt) + (1.0f - alpha) * velocity_right;
    }
}

// =============================================================================
// Odometrie
// =============================================================================

void odometry_update(float dt) {
    float d_left = velocity_left * dt;
    float d_right = velocity_right * dt;

    float d_center = (d_left + d_right) / 2.0f;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    odom_x += d_center * cos(odom_theta + d_theta / 2.0f);
    odom_y += d_center * sin(odom_theta + d_theta / 2.0f);
    odom_theta += d_theta;

    while (odom_theta > PI)
        odom_theta -= 2.0f * PI;
    while (odom_theta < -PI)
        odom_theta += 2.0f * PI;
}

// =============================================================================
// Kinematik & Motor Control
// =============================================================================

void kinematics_inverse(float v, float w, float *v_left, float *v_right) {
    *v_left = v - (w * WHEEL_BASE / 2.0f);
    *v_right = v + (w * WHEEL_BASE / 2.0f);
}

void motor_control_update(float dt) {
    kinematics_inverse(target_linear, target_angular, &target_v_left,
                       &target_v_right);

    target_v_left =
        constrain(target_v_left, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    target_v_right =
        constrain(target_v_right, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

    if (fabs(target_v_left) < 0.01f && fabs(target_v_right) < 0.01f) {
        stopMotors();
        pid_left.reset();
        pid_right.reset();
        return;
    }

    float pwm_left = pid_left.compute(target_v_left, velocity_left, dt);
    float pwm_right = pid_right.compute(target_v_right, velocity_right, dt);

    hal_motor_set_pwm(PWM_CH_LEFT_A, PWM_CH_LEFT_B, pwm_left);
    hal_motor_set_pwm(PWM_CH_RIGHT_A, PWM_CH_RIGHT_B, pwm_right);
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
// Failsafe (1000ms Timeout für DDS Discovery)
// =============================================================================

void checkFailsafe() {
    if (millis() - last_cmd_time > 1000) {
        target_linear = 0.0f;
        target_angular = 0.0f;
        stopMotors();
        pid_left.reset();
        pid_right.reset();
    }
}

// =============================================================================
// Odometry Publish
// =============================================================================

void publish_odometry() {
    msg_odom_x.data = odom_x;
    msg_odom_y.data = odom_y;
    msg_odom_theta.data = odom_theta;

    (void)rcl_publish(&pub_odom_x, &msg_odom_x, NULL);
    (void)rcl_publish(&pub_odom_y, &msg_odom_y, NULL);
    (void)rcl_publish(&pub_odom_theta, &msg_odom_theta, NULL);
}

void setStatusLED(bool on) { digitalWrite(LED_PIN, on ? LOW : HIGH); }

// =============================================================================
// Setup
// =============================================================================

void setup() {
    pinMode(LED_PIN, OUTPUT);
    setStatusLED(false);

    pinMode(PIN_LED_MOSFET, OUTPUT);
    digitalWrite(PIN_LED_MOSFET, LOW);

    setupMotors();
    hal_encoder_init();

    Serial.begin(115200);
    delay(2000);

    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();

    // Warte auf Agent
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        setStatusLED((millis() / 250) % 2);
        delay(100);
    }
    setStatusLED(true);

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Publisher (4 Topics)
    rclc_publisher_init_best_effort(
        &pub_heartbeat, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32/heartbeat");

    rclc_publisher_init_best_effort(
        &pub_odom_x, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "odom/x");

    rclc_publisher_init_best_effort(
        &pub_odom_y, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "odom/y");

    rclc_publisher_init_best_effort(
        &pub_odom_theta, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "odom/theta");

    // Subscriber (reliable für cmd_vel)
    rclc_subscription_init_default(
        &sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    rclc_subscription_init_default(
        &sub_led_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "esp32/led_cmd");

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
                                   &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &sub_led_cmd, &msg_led_cmd,
                                   &led_callback, ON_NEW_DATA);

    last_cmd_time = millis();
    last_loop_time = millis();
    last_odom_time = millis();
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
    unsigned long now = millis();

    // micro-ROS Executor (schneller Spin für besseren Empfang)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

    // Hauptschleife (100 Hz)
    if (now - last_loop_time >= LOOP_PERIOD_MS) {
        float dt = (now - last_loop_time) / 1000.0f;
        last_loop_time = now;

        velocity_update(dt);
        odometry_update(dt);
        checkFailsafe();
        motor_control_update(dt);
    }

    // Odometrie publizieren (20 Hz - reduziert für Serial-Bandbreite)
    if (now - last_odom_time >= 50) {
        last_odom_time = now;
        publish_odometry();
    }

    // Heartbeat (1 Hz)
    static unsigned long last_heartbeat = 0;
    if (now - last_heartbeat > 1000) {
        msg_heartbeat.data = heartbeat_counter++;
        (void)rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL);
        last_heartbeat = now;
    }
}
