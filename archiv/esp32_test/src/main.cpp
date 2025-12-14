/**
 * @file main.cpp
 * @brief ESP32-S3 XIAO Standard-Testprogramm
 * @version 1.0.1
 * @date 2025-12-12
 *
 * LED-Strip an D10 über MOSFET IRLZ24N
 * Effekt: Sanftes Breathing (Auf- und Abdimmen)
 *
 * @standard Non-Blocking Code (keine delay() im Loop)
 */

#include "config.h"
#include <Arduino.h>

// Breathing-Parameter
#define BREATH_MIN 0       // Minimale Helligkeit
#define BREATH_MAX 255     // Maximale Helligkeit
#define BREATH_STEP 5      // Schrittweite
#define BREATH_INTERVAL 30 // ms zwischen Schritten (~3s pro Zyklus)

int brightness = 0;
int direction = 1; // 1 = heller, -1 = dunkler
unsigned long lastBreathTime = 0;

void setup() {
    Serial.begin(DEBUG_BAUD);
    delay(1000); // USB-CDC Boot abwarten (nur im Setup erlaubt)

    Serial.println();
    Serial.println("================================");
    Serial.println("  ESP32-S3 XIAO Testprogramm");
    Serial.println("================================");
    Serial.printf("LED Pin: D10 (GPIO %d)\n", PIN_LED_MOSFET);
    Serial.printf("PWM Kanal: %d\n", LED_PWM_CHANNEL);
    Serial.printf("PWM Frequenz: %d Hz\n", LED_PWM_FREQ);
    Serial.println("Effekt: Breathing (Non-Blocking)");
    Serial.println();

    // PWM Setup (ESP32 Arduino 2.x API)
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_BITS);
    ledcAttachPin(PIN_LED_MOSFET, LED_PWM_CHANNEL);

    Serial.println("System bereit.");
}

void loop() {
    unsigned long now = millis();

    // Non-Blocking Timer für LED-Breathing
    if (now - lastBreathTime >= BREATH_INTERVAL) {
        lastBreathTime = now;

        // Helligkeit anpassen
        brightness += direction * BREATH_STEP;

        // Richtung umkehren an den Grenzen
        if (brightness >= BREATH_MAX) {
            brightness = BREATH_MAX;
            direction = -1;
        } else if (brightness <= BREATH_MIN) {
            brightness = BREATH_MIN;
            direction = 1;
        }

        // LED setzen
        ledcWrite(LED_PWM_CHANNEL, brightness);
    }

    // Hier können weitere non-blocking Tasks laufen
    // z.B. Serial-Kommandos verarbeiten, Sensoren lesen, etc.
}
