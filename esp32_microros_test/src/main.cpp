// =============================================================================
// micro-ROS Test-Firmware für Seeed XIAO ESP32-S3
// Angepasst an config.h des AMR-Projekts
// =============================================================================
//
// Publisher:  /esp32/heartbeat  (std_msgs/Int32) - Zähler alle 500ms
// Subscriber: /esp32/led        (std_msgs/Int32) - LED-Steuerung (0=aus, 1=an)
//
// Hardware: Seeed Studio XIAO ESP32-S3
// LED: Interne LED (GPIO 21) oder externe über MOSFET (D10)
// =============================================================================

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>

// -----------------------------------------------------------------------------
// Konfiguration für XIAO ESP32-S3
// -----------------------------------------------------------------------------
// XIAO ESP32-S3 interne LED: GPIO 21 (active LOW!)
// Alternative: D10 über MOSFET (active HIGH) aus config.h
#define LED_PIN 21          // Interne LED auf XIAO ESP32-S3
#define LED_ACTIVE_LOW true // XIAO interne LED ist active LOW

#define HEARTBEAT_INTERVAL_MS 500 // Publisher-Intervall
#define AGENT_TIMEOUT_MS 2000     // Timeout für Agent-Verbindung

// -----------------------------------------------------------------------------
// micro-ROS Objekte
// -----------------------------------------------------------------------------
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 pub_msg;
std_msgs__msg__Int32 sub_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// -----------------------------------------------------------------------------
// Zustandsvariablen
// -----------------------------------------------------------------------------
enum State {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

State state = WAITING_AGENT;
int32_t heartbeat_counter = 0;
unsigned long last_heartbeat = 0;

// -----------------------------------------------------------------------------
// Fehlerbehandlung
// -----------------------------------------------------------------------------
#define RCCHECK(fn)                                                            \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK)) {                                         \
            return false;                                                      \
        }                                                                      \
    }
#define RCSOFTCHECK(fn)                                                        \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK)) {                                         \
        }                                                                      \
    }

// -----------------------------------------------------------------------------
// LED Helper (invertiert für active-low)
// -----------------------------------------------------------------------------
void setLED(bool on) {
    if (LED_ACTIVE_LOW) {
        digitalWrite(LED_PIN, on ? LOW : HIGH);
    } else {
        digitalWrite(LED_PIN, on ? HIGH : LOW);
    }
}

// -----------------------------------------------------------------------------
// Callback: LED-Steuerung empfangen
// -----------------------------------------------------------------------------
void led_callback(const void *msgin) {
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    setLED(msg->data != 0);
}

// -----------------------------------------------------------------------------
// Callback: Timer für Heartbeat
// -----------------------------------------------------------------------------
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        pub_msg.data = heartbeat_counter++;
        RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
    }
}

// -----------------------------------------------------------------------------
// micro-ROS Entities erstellen
// -----------------------------------------------------------------------------
bool create_entities() {
    allocator = rcl_get_default_allocator();

    // Support initialisieren
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Node erstellen
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));

    // Publisher erstellen
    RCCHECK(rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32/heartbeat"));

    // Subscriber erstellen
    RCCHECK(rclc_subscription_init_default(
        &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32/led"));

// Timer erstellen (500ms)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(HEARTBEAT_INTERVAL_MS), timer_callback));
#pragma GCC diagnostic pop

    // Executor erstellen
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
                                           &led_callback, ON_NEW_DATA));

    return true;
}

// -----------------------------------------------------------------------------
// micro-ROS Entities aufräumen
// -----------------------------------------------------------------------------
void destroy_entities() {
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void)rcl_publisher_fini(&publisher, &node);
    (void)rcl_subscription_fini(&subscriber, &node);
    (void)rcl_timer_fini(&timer);
    (void)rclc_executor_fini(&executor);
    (void)rcl_node_fini(&node);
    (void)rclc_support_fini(&support);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
    // LED initialisieren
    pinMode(LED_PIN, OUTPUT);
    setLED(false);

    // micro-ROS Transport konfigurieren (Serial)
    // WICHTIG: Kein Serial.begin() oder Serial.print() verwenden!
    // Serial wird exklusiv von micro-ROS genutzt.
    set_microros_serial_transports(Serial);

    // Initialer Zustand
    state = WAITING_AGENT;
}

// -----------------------------------------------------------------------------
// Loop (Zustandsmaschine)
// -----------------------------------------------------------------------------
void loop() {
    switch (state) {
    // -----------------------------------------------------------------
    case WAITING_AGENT:
        // LED blinken während Warten (schnell = 250ms)
        setLED((millis() / 250) % 2);

        // Prüfen ob Agent verfügbar
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
            state = AGENT_AVAILABLE;
        }
        break;

    // -----------------------------------------------------------------
    case AGENT_AVAILABLE:
        if (create_entities()) {
            setLED(true); // LED an = verbunden
            state = AGENT_CONNECTED;
        } else {
            state = WAITING_AGENT;
            destroy_entities();
        }
        break;

    // -----------------------------------------------------------------
    case AGENT_CONNECTED:
        // Executor ausführen (nicht-blockierend)
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

        // Periodisch Agent-Verbindung prüfen
        if (millis() - last_heartbeat > AGENT_TIMEOUT_MS) {
            last_heartbeat = millis();
            if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
                state = AGENT_DISCONNECTED;
            }
        }
        break;

    // -----------------------------------------------------------------
    case AGENT_DISCONNECTED:
        destroy_entities();
        setLED(false);
        state = WAITING_AGENT;
        break;
    }

    // Kurze Pause
    delay(10);
}
