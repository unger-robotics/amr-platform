/**
 * @file main.cpp
 * @brief AMR Hardware-Test Firmware (OHNE micro-ROS)
 * @version 0.1.0-test
 * 
 * Testet:
 * - LED-Breathing (D10)
 * - Motor-PWM (D0-D3)
 * - Serial-Kommunikation
 * 
 * Befehle über Serial Monitor (115200 Baud):
 *   'f' = Forward (beide Motoren vorwärts)
 *   'b' = Backward (beide Motoren rückwärts)
 *   'l' = Left (Drehung links)
 *   'r' = Right (Drehung rechts)
 *   's' = Stop
 *   '1'-'9' = Geschwindigkeit (10%-90%)
 */

#include <Arduino.h>

// ==========================================================================
// PIN-DEFINITIONEN (wie in config.h)
// ==========================================================================

// Motor Pins (Cytron MDD3A - Dual PWM)
#define PIN_MOTOR_LEFT_A    D0  // M1A - Vorwärts
#define PIN_MOTOR_LEFT_B    D1  // M1B - Rückwärts
#define PIN_MOTOR_RIGHT_A   D2  // M2A - Vorwärts
#define PIN_MOTOR_RIGHT_B   D3  // M2B - Rückwärts

// LED Pin
#define PIN_LED_MOSFET      D10

// PWM Kanäle
#define PWM_CH_LEFT_A       0
#define PWM_CH_LEFT_B       1
#define PWM_CH_RIGHT_A      2
#define PWM_CH_RIGHT_B      3
#define PWM_CH_LED          4

// PWM Konfiguration
#define PWM_FREQ            20000  // 20 kHz
#define PWM_BITS            8      // 0-255
#define LED_PWM_FREQ        5000

// ==========================================================================
// GLOBALE VARIABLEN
// ==========================================================================

int currentSpeed = 128;  // 50% default
bool motorsEnabled = false;

// ==========================================================================
// MOTOR FUNKTIONEN
// ==========================================================================

void motorSetup() {
    // PWM Kanäle konfigurieren
    ledcSetup(PWM_CH_LEFT_A, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_LEFT_B, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_A, PWM_FREQ, PWM_BITS);
    ledcSetup(PWM_CH_RIGHT_B, PWM_FREQ, PWM_BITS);
    
    // Pins zuweisen
    ledcAttachPin(PIN_MOTOR_LEFT_A, PWM_CH_LEFT_A);
    ledcAttachPin(PIN_MOTOR_LEFT_B, PWM_CH_LEFT_B);
    ledcAttachPin(PIN_MOTOR_RIGHT_A, PWM_CH_RIGHT_A);
    ledcAttachPin(PIN_MOTOR_RIGHT_B, PWM_CH_RIGHT_B);
    
    // Alle Motoren stoppen
    ledcWrite(PWM_CH_LEFT_A, 0);
    ledcWrite(PWM_CH_LEFT_B, 0);
    ledcWrite(PWM_CH_RIGHT_A, 0);
    ledcWrite(PWM_CH_RIGHT_B, 0);
    
    Serial.println("[MOTOR] Initialisiert (D0-D3)");
}

void motorLeft(int speed) {
    // speed > 0 = vorwärts, speed < 0 = rückwärts
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
    motorsEnabled = false;
    Serial.println("[MOTOR] STOP");
}

void motorForward(int speed) {
    motorLeft(speed);
    motorRight(speed);
    motorsEnabled = true;
    Serial.printf("[MOTOR] FORWARD speed=%d\n", speed);
}

void motorBackward(int speed) {
    motorLeft(-speed);
    motorRight(-speed);
    motorsEnabled = true;
    Serial.printf("[MOTOR] BACKWARD speed=%d\n", speed);
}

void motorTurnLeft(int speed) {
    motorLeft(-speed);
    motorRight(speed);
    motorsEnabled = true;
    Serial.printf("[MOTOR] TURN LEFT speed=%d\n", speed);
}

void motorTurnRight(int speed) {
    motorLeft(speed);
    motorRight(-speed);
    motorsEnabled = true;
    Serial.printf("[MOTOR] TURN RIGHT speed=%d\n", speed);
}

// ==========================================================================
// LED FUNKTIONEN
// ==========================================================================

void ledSetup() {
    ledcSetup(PWM_CH_LED, LED_PWM_FREQ, PWM_BITS);
    ledcAttachPin(PIN_LED_MOSFET, PWM_CH_LED);
    Serial.println("[LED] Initialisiert (D10)");
}

void ledBreathing() {
    // Breathing-Effekt (nicht-blockierend)
    static uint32_t lastUpdate = 0;
    static int brightness = 0;
    static int direction = 1;
    
    if (millis() - lastUpdate > 10) {  // 10ms Update
        lastUpdate = millis();
        brightness += direction * 3;
        
        if (brightness >= 255) {
            brightness = 255;
            direction = -1;
        } else if (brightness <= 0) {
            brightness = 0;
            direction = 1;
        }
        
        ledcWrite(PWM_CH_LED, brightness);
    }
}

void ledBlink(int onTime, int offTime) {
    static uint32_t lastChange = 0;
    static bool state = false;
    
    uint32_t interval = state ? onTime : offTime;
    
    if (millis() - lastChange > interval) {
        lastChange = millis();
        state = !state;
        ledcWrite(PWM_CH_LED, state ? 255 : 0);
    }
}

// ==========================================================================
// SERIAL COMMAND HANDLER
// ==========================================================================

void handleSerial() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'f':
            case 'F':
                motorForward(currentSpeed);
                break;
                
            case 'b':
            case 'B':
                motorBackward(currentSpeed);
                break;
                
            case 'l':
            case 'L':
                motorTurnLeft(currentSpeed);
                break;
                
            case 'r':
            case 'R':
                motorTurnRight(currentSpeed);
                break;
                
            case 's':
            case 'S':
            case ' ':
                motorStop();
                break;
                
            case '1': currentSpeed = 25;  Serial.printf("[SPEED] 10%% (%d)\n", currentSpeed); break;
            case '2': currentSpeed = 51;  Serial.printf("[SPEED] 20%% (%d)\n", currentSpeed); break;
            case '3': currentSpeed = 76;  Serial.printf("[SPEED] 30%% (%d)\n", currentSpeed); break;
            case '4': currentSpeed = 102; Serial.printf("[SPEED] 40%% (%d)\n", currentSpeed); break;
            case '5': currentSpeed = 128; Serial.printf("[SPEED] 50%% (%d)\n", currentSpeed); break;
            case '6': currentSpeed = 153; Serial.printf("[SPEED] 60%% (%d)\n", currentSpeed); break;
            case '7': currentSpeed = 179; Serial.printf("[SPEED] 70%% (%d)\n", currentSpeed); break;
            case '8': currentSpeed = 204; Serial.printf("[SPEED] 80%% (%d)\n", currentSpeed); break;
            case '9': currentSpeed = 230; Serial.printf("[SPEED] 90%% (%d)\n", currentSpeed); break;
            
            case 'h':
            case 'H':
            case '?':
                Serial.println("\n=== AMR Hardware Test ===");
                Serial.println("Befehle:");
                Serial.println("  f = Forward");
                Serial.println("  b = Backward");
                Serial.println("  l = Turn Left");
                Serial.println("  r = Turn Right");
                Serial.println("  s = Stop");
                Serial.println("  1-9 = Speed (10%-90%)");
                Serial.println("  h = Help");
                Serial.printf("\nAktuelle Geschwindigkeit: %d\n", currentSpeed);
                break;
                
            default:
                break;
        }
    }
}

// ==========================================================================
// SETUP & LOOP
// ==========================================================================

void setup() {
    Serial.begin(115200);
    delay(2000);  // Warten auf Serial
    
    Serial.println("\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║   AMR Hardware Test v0.1.0             ║");
    Serial.println("║   ESP32-S3 XIAO + Cytron MDD3A         ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println("");
    
    motorSetup();
    ledSetup();
    
    Serial.println("\n[READY] Befehle: f/b/l/r/s, 1-9 für Speed, h für Hilfe\n");
}

void loop() {
    // LED-Effekt basierend auf Motor-Status
    if (motorsEnabled) {
        ledBlink(100, 100);  // Schnelles Blinken wenn Motoren aktiv
    } else {
        ledBreathing();      // Breathing im Idle
    }
    
    // Serial-Befehle verarbeiten
    handleSerial();
    
    // Kleine Pause
    delay(1);
}
