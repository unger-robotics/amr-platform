/**
 * @file main.cpp
 * @brief AMR Automatischer Motor-Test
 * @version 0.2.0-test
 *
 * Automatischer Testlauf bei 20% Speed:
 * - Forward 2s
 * - Stop 1s
 * - Backward 2s
 * - Stop 1s
 * - Turn Left 2s
 * - Stop 1s
 * - Turn Right 2s
 * - Stop 1s
 * - FERTIG
 */

#include <Arduino.h>

// ==========================================================================
// PIN-DEFINITIONEN
// ==========================================================================

#define PIN_MOTOR_LEFT_A D0
#define PIN_MOTOR_LEFT_B D1
#define PIN_MOTOR_RIGHT_A D2
#define PIN_MOTOR_RIGHT_B D3
#define PIN_LED_MOSFET D10

#define PWM_CH_LEFT_A 0
#define PWM_CH_LEFT_B 1
#define PWM_CH_RIGHT_A 2
#define PWM_CH_RIGHT_B 3
#define PWM_CH_LED 4

#define PWM_FREQ 20000
#define PWM_BITS 8
#define LED_PWM_FREQ 5000

// ==========================================================================
// TEST PARAMETER
// ==========================================================================

#define TEST_SPEED 51          // 20% von 255
#define MOVE_DURATION_MS 2000  // 2 Sekunden pro Bewegung
#define PAUSE_DURATION_MS 1000 // 1 Sekunde Pause

// ==========================================================================
// MOTOR FUNKTIONEN
// ==========================================================================

void motorSetup() {
    ledcSetup(PWM_CH_LEFT_A, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, PWM_FREQ, PWM_BITS);

    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);

    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

void motorLeft(int speed) {
    if (speed >= 0) {
        ledcWrite(PWM_CH_LEFT_A, speed);
        ledcWrite(PWM_CH_LEFT_B, 0);
    } else {
        ledcWrite(PWM_CH_LEFT_A, 0);
        ledcWrite(PWM_CH_LEFT_B, -speed);
    }
}

void motorRight(int speed) {
    if (speed >= 0) {
        ledcWrite(PWM_CH_RIGHT_A, speed);
        ledcWrite(PWM_CH_RIGHT_B, 0);
    } else {
        ledcWrite(PWM_CH_RIGHT_A, 0);
        ledcWrite(PWM_CH_RIGHT_B, -speed);
    }
}

void motorStop() {
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
}

void motorForward(int speed) {
    motorLeft(speed);
    motorRight(speed);
}

void motorBackward(int speed) {
    motorLeft(-speed);
    motorRight(-speed);
}

void motorTurnLeft(int speed) {
    motorLeft(-speed);
    motorRight(speed);
}

void motorTurnRight(int speed) {
    motorLeft(speed);
    motorRight(-speed);
}

// ==========================================================================
// LED FUNKTIONEN
// ==========================================================================

void ledSetup() {
    ledcSetup(PWM_CH_LED, LED_PWM_FREQ, PWM_BITS);
    ledcAttachPin(PIN_LED_MOSFET, PWM_CH_LED);
}

void ledOn() { ledcWrite(PWM_CH_LED, 255); }

void ledOff() { ledcWrite(PWM_CH_LED, 0); }

// ==========================================================================
// TEST SEQUENZ
// ==========================================================================

void runTestSequence() {
    Serial.println("\n========================================");
    Serial.println("    AUTOMATISCHER MOTOR-TEST");
    Serial.printf("    Speed: %d (20%%)\n", TEST_SPEED);
    Serial.println("========================================\n");

    // Countdown
    Serial.println("Start in 3...");
    delay(1000);
    Serial.println("Start in 2...");
    delay(1000);
    Serial.println("Start in 1...");
    delay(1000);
    Serial.println("LOS!\n");

    // Test 1: FORWARD
    Serial.println("[TEST 1/4] FORWARD - 2 Sekunden");
    ledOn();
    motorForward(TEST_SPEED);
    delay(MOVE_DURATION_MS);
    motorStop();
    ledOff();
    Serial.println("          STOP - 1 Sekunde Pause\n");
    delay(PAUSE_DURATION_MS);

    // Test 2: BACKWARD
    Serial.println("[TEST 2/4] BACKWARD - 2 Sekunden");
    ledOn();
    motorBackward(TEST_SPEED);
    delay(MOVE_DURATION_MS);
    motorStop();
    ledOff();
    Serial.println("          STOP - 1 Sekunde Pause\n");
    delay(PAUSE_DURATION_MS);

    // Test 3: TURN LEFT
    Serial.println("[TEST 3/4] TURN LEFT - 2 Sekunden");
    ledOn();
    motorTurnLeft(TEST_SPEED);
    delay(MOVE_DURATION_MS);
    motorStop();
    ledOff();
    Serial.println("          STOP - 1 Sekunde Pause\n");
    delay(PAUSE_DURATION_MS);

    // Test 4: TURN RIGHT
    Serial.println("[TEST 4/4] TURN RIGHT - 2 Sekunden");
    ledOn();
    motorTurnRight(TEST_SPEED);
    delay(MOVE_DURATION_MS);
    motorStop();
    ledOff();
    Serial.println("          STOP\n");

    // Fertig
    Serial.println("========================================");
    Serial.println("    TEST ABGESCHLOSSEN!");
    Serial.println("========================================");
    Serial.println("\nErwartetes Verhalten:");
    Serial.println("  1. Beide Raeder vorwaerts");
    Serial.println("  2. Beide Raeder rueckwaerts");
    Serial.println("  3. Links rueck, Rechts vor (Linksdrehung)");
    Serial.println("  4. Links vor, Rechts rueck (Rechtsdrehung)");
    Serial.println("\nDruecke RESET fuer erneuten Test.");
}

// ==========================================================================
// SETUP & LOOP
// ==========================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║   AMR Motor Test v0.2.0                ║");
    Serial.println("║   20% Speed - Automatisch              ║");
    Serial.println("╚════════════════════════════════════════╝");

    motorSetup();
    ledSetup();

    // Automatisch starten
    runTestSequence();
}

void loop() {
    // Nichts - Test läuft nur einmal in setup()
    delay(1000);
}
