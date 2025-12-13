#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include "config.h"

// --- KORREKTUR: Definition wieder hinzugefügt ---
#define LED_PIN 21 // Die gelbe Onboard-LED des XIAO ESP32-S3

// Globale Objekte
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;

// Nachrichten
std_msgs__msg__Int32 msg_heartbeat;
std_msgs__msg__Bool msg_led_cmd;

// --- CALLBACK FUNKTION ---
void led_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    // Pin D10 (aus config.h) schalten
    digitalWrite(PIN_LED_MOSFET, msg->data ? HIGH : LOW);
}

void setup() {
    Serial.begin(115200);

    // Status LED (Onboard)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // MOSFET (Externer Streifen)
    pinMode(PIN_LED_MOSFET, OUTPUT);
    digitalWrite(PIN_LED_MOSFET, LOW);

    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();

    // Verbindung mit Blinken anzeigen
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
    digitalWrite(LED_PIN, LOW); // Verbunden

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_node", "", &support);

    // Publisher (Heartbeat)
    rclc_publisher_init_best_effort(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "esp32/heartbeat");

    // Subscriber (LED Befehl)
    rclc_subscription_init_default(
        &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "esp32/led_cmd");

    // Executor
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg_led_cmd,
                                   &led_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat > 1000) {
        msg_heartbeat.data++;
        // Warnung unterdrücken durch Cast auf (void)
        (void)rcl_publish(&publisher, &msg_heartbeat, NULL);
        last_heartbeat = millis();
    }
}
