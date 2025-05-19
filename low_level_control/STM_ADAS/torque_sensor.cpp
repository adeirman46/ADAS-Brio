#include <Arduino.h>

// Definisi pin untuk ADC
#define ANALOG1 A0
#define ANALOG2 A1

// Definisi pin untuk output DAC
#define DAC1 PA4
#define DAC2 PA5

// Mode Kontrol
bool manualMode = true; // Default: Manual Mode

// Nilai idle state 2.5V pada resolusi 10-bit (2.5V / 3.3V * 1023)
#define IDLE_STATE 775

// Variabel tegangan untuk mode keyboard
float voltage1 = 2.5;
float voltage2 = 2.5;

// Variabel untuk smoothing di manual mode
static float smoothed_val1 = IDLE_STATE;
static float smoothed_val2 = IDLE_STATE;
const float alpha = 0.04; // Faktor smoothing (0–1, kecil = lebih halus)

void setup() {
    Serial.begin(9600);
    analogReadResolution(10); // Resolusi ADC ke 10-bit
    analogWriteResolution(10); // Resolusi DAC ke 10-bit
    
    // Set DAC ke idle state awal
    analogWrite(DAC1, IDLE_STATE);
    analogWrite(DAC2, IDLE_STATE);
    
    Serial.println("Mode: Manual (default)");
    Serial.println("Tekan 'm' untuk beralih ke mode Keyboard");
}

void loop() {
    if (Serial.available()) {
        char input = Serial.read();
        
        if (input == 'm' || input == 'M') {
            manualMode = !manualMode;
            if (!manualMode) {
                voltage1 = 2.5;
                voltage2 = 2.5;
            }
            Serial.print("Mode: ");
            Serial.println(manualMode ? "Manual" : "Keyboard");
        }
        
        if (!manualMode) {
            if (input == 'a' || input == 'A') {
                if (voltage1 < 3.3 && voltage2 > 0.0) {
                    voltage1 += 0.1;
                    voltage2 -= 0.1;
                }
            } else if (input == 'd' || input == 'D') {
                if (voltage1 > 0.0 && voltage2 < 3.3) {
                    voltage1 -= 0.1;
                    voltage2 += 0.1;
                }
            }
            // Set DAC dalam mode keyboard (10-bit)
            analogWrite(DAC1, (voltage1 / 3.3) * 1023);
            analogWrite(DAC2, (voltage2 / 3.3) * 1023);
            
            Serial.print("DAC Output - PA4: ");
            Serial.print(voltage1, 2);
            Serial.print("V, PA5: ");
            Serial.print(voltage2, 2);
            Serial.println("V");
        }
    }
    
    if (manualMode) {
        int val1 = analogRead(ANALOG1); // 0–1023
        int val2 = analogRead(ANALOG2);
        
        // Terapkan low-pass filter
        smoothed_val1 = (1 - alpha) * smoothed_val1 + alpha * val1;
        smoothed_val2 = (1 - alpha) * smoothed_val2 + alpha * val2;
        
        // Tulis ke DAC
        analogWrite(DAC1, val1 > 0 ? (int)smoothed_val1 : IDLE_STATE);
        analogWrite(DAC2, val2 > 0 ? (int)smoothed_val2 : IDLE_STATE);
    }
    
    delay(10);
}