// Minimaler USB-CDC Test für XIAO ESP32-S3
#include <Arduino.h>

#define LED_PIN 21

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // USB-CDC initialisieren
    Serial.begin(115200);

    // Warten bis USB bereit
    delay(3000);
}

void loop() {
    // LED toggeln
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    // Text senden
    Serial.println("Hello from ESP32");

    delay(1000);
}
