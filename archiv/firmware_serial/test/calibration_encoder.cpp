/**
 * @file calibration_encoder.cpp
 * @brief Encoder-Kalibrierung für AMR Phase 2
 * @version 1.0.0
 * @date 2025-12-12
 * 
 * VERWENDUNG:
 * 1. Diese Datei temporär als main.cpp verwenden:
 *    mv src/main.cpp src/main.cpp.backup
 *    cp calibration_encoder.cpp src/main.cpp
 * 
 * 2. Flashen: pio run -t upload
 * 3. Serial Monitor: pio device monitor
 * 
 * 4. Kalibrierung durchführen:
 *    a) Rad markieren (Strich auf Reifen + Chassis ausrichten)
 *    b) 'r' drücken zum Reset der Zähler
 *    c) Rad EXAKT 10 Umdrehungen von Hand drehen
 *    d) Ticks ablesen und durch 10 teilen
 *    e) Für beide Räder einzeln durchführen!
 * 
 * 5. Werte in config.h eintragen:
 *    #define TICKS_PER_REV_LEFT   xxx.xf
 *    #define TICKS_PER_REV_RIGHT  xxx.xf
 * 
 * 6. Original wiederherstellen:
 *    mv src/main.cpp.backup src/main.cpp
 * 
 * @note Pin-Belegung aus config.h:
 *       PIN_ENC_LEFT_A  = D6
 *       PIN_ENC_RIGHT_A = D7
 */

#include <Arduino.h>

// ==========================================================================
// PIN-KONFIGURATION (identisch zu config.h)
// ==========================================================================
#define PIN_ENC_LEFT_A    D6    // Interrupt-fähig
#define PIN_ENC_RIGHT_A   D7    // Interrupt-fähig

// ==========================================================================
// GLOBALE VARIABLEN
// ==========================================================================
volatile long ticks_left = 0;
volatile long ticks_right = 0;

// Für Richtungserkennung (optional, falls Encoder B-Kanal vorhanden)
volatile int8_t dir_left = 1;
volatile int8_t dir_right = 1;

// ==========================================================================
// INTERRUPT SERVICE ROUTINES
// ==========================================================================

/**
 * @brief ISR für linken Encoder
 * @note IRAM_ATTR: Code im RAM für schnellere Ausführung
 */
void IRAM_ATTR isr_encoder_left() {
    ticks_left++;
}

/**
 * @brief ISR für rechten Encoder
 */
void IRAM_ATTR isr_encoder_right() {
    ticks_right++;
}

// ==========================================================================
// SETUP
// ==========================================================================
void setup() {
    // Serial initialisieren
    Serial.begin(115200);
    
    // Warten auf Serial-Verbindung (USB-CDC)
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {
        delay(10);
    }
    
    // Encoder-Pins konfigurieren
    pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
    pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
    
    // Interrupts aktivieren (RISING = steigende Flanke)
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), 
                    isr_encoder_left, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), 
                    isr_encoder_right, RISING);
    
    // Begrüßung
    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════╗");
    Serial.println("║           AMR ENCODER KALIBRIERUNG - Phase 2               ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("║  Pin-Belegung:                                             ║");
    Serial.println("║    Encoder Links:  D6                                      ║");
    Serial.println("║    Encoder Rechts: D7                                      ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("║  Anleitung:                                                ║");
    Serial.println("║  1. Rad markieren (Strich auf Reifen + Chassis)            ║");
    Serial.println("║  2. 'r' drücken zum Reset der Zähler                       ║");
    Serial.println("║  3. Rad EXAKT 10 Umdrehungen drehen                        ║");
    Serial.println("║  4. Ticks ablesen → durch 10 teilen = TICKS_PER_REV        ║");
    Serial.println("║  5. Für BEIDE Räder einzeln durchführen!                   ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("║  Befehle:                                                  ║");
    Serial.println("║    'r' = Reset beider Zähler                               ║");
    Serial.println("║    'l' = Reset nur links                                   ║");
    Serial.println("║    'R' = Reset nur rechts                                  ║");
    Serial.println("║    'c' = Berechnung anzeigen (nach 10 Umdrehungen)         ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Warte auf Encoder-Impulse...");
    Serial.println();
}

// ==========================================================================
// HAUPTSCHLEIFE
// ==========================================================================
void loop() {
    static unsigned long last_print = 0;
    static long last_ticks_left = 0;
    static long last_ticks_right = 0;
    
    // Alle 500ms Status ausgeben
    if (millis() - last_print >= 500) {
        // Nur ausgeben wenn sich etwas geändert hat oder regelmäßig
        long current_left = ticks_left;
        long current_right = ticks_right;
        
        // Delta seit letztem Print (für Aktivitätsanzeige)
        long delta_left = current_left - last_ticks_left;
        long delta_right = current_right - last_ticks_right;
        
        // Aktivitätsindikator
        char activity_left = (delta_left > 0) ? '*' : ' ';
        char activity_right = (delta_right > 0) ? '*' : ' ';
        
        Serial.printf("LINKS: %6ld ticks %c | RECHTS: %6ld ticks %c", 
                      current_left, activity_left,
                      current_right, activity_right);
        
        // Wenn genug Ticks für Berechnung
        if (current_left >= 100 || current_right >= 100) {
            Serial.printf("  |  /10 → L: %.1f  R: %.1f", 
                          current_left / 10.0f,
                          current_right / 10.0f);
        }
        Serial.println();
        
        last_ticks_left = current_left;
        last_ticks_right = current_right;
        last_print = millis();
    }
    
    // Serial-Befehle verarbeiten
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'r':  // Reset beide
                noInterrupts();
                ticks_left = 0;
                ticks_right = 0;
                interrupts();
                Serial.println();
                Serial.println(">>> RESET: Beide Zähler auf 0 <<<");
                Serial.println();
                break;
                
            case 'l':  // Reset nur links
                noInterrupts();
                ticks_left = 0;
                interrupts();
                Serial.println(">>> RESET: Links auf 0 <<<");
                break;
                
            case 'R':  // Reset nur rechts (großes R)
                noInterrupts();
                ticks_right = 0;
                interrupts();
                Serial.println(">>> RESET: Rechts auf 0 <<<");
                break;
                
            case 'c':  // Berechnung anzeigen
            case 'C':
                Serial.println();
                Serial.println("═══════════════════════════════════════════════");
                Serial.println("KALIBRIERUNGS-ERGEBNIS (bei 10 Umdrehungen):");
                Serial.println("───────────────────────────────────────────────");
                Serial.printf("  TICKS_PER_REV_LEFT  = %.1ff\n", ticks_left / 10.0f);
                Serial.printf("  TICKS_PER_REV_RIGHT = %.1ff\n", ticks_right / 10.0f);
                Serial.println("───────────────────────────────────────────────");
                Serial.println("Diese Werte in config.h eintragen!");
                Serial.println("═══════════════════════════════════════════════");
                Serial.println();
                break;
                
            case '?':  // Hilfe
            case 'h':
            case 'H':
                Serial.println();
                Serial.println("Befehle: r=Reset alle, l=Reset links, R=Reset rechts, c=Berechnung");
                Serial.println();
                break;
        }
    }
    
    // Minimale Verzögerung für Stabilität
    delay(10);
}
