#include <Arduino.h>

// Definisi pin untuk ADC
#define ANALOG1 A0
#define ANALOG2 A1

// Definisi pin untuk output DAC
#define DAC1 PA4
#define DAC2 PA5

// Mode Kontrol
bool manualMode = true; // Default: Manual Mode

// Nilai idle state 2.5V pada resolusi 12-bit (2.5V / 3.3V * 4095)
#define IDLE_STATE 3100

// Variabel tegangan untuk mode keyboard
float voltage1 = 2.5;
float voltage2 = 2.5;

void setup() {
    Serial.begin(9600);
    analogReadResolution(12); // Mengatur resolusi ADC ke 12-bit
    analogWriteResolution(12); // Mengatur resolusi DAC ke 12-bit
    
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
            
            // Reset voltage saat beralih ke mode keyboard
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
            
            // Set DAC dalam mode keyboard
            analogWrite(DAC1, (voltage1 / 3.3) * 4095);
            analogWrite(DAC2, (voltage2 / 3.3) * 4095);
            
            // Print status
            Serial.print("DAC Output - PA4: ");
            Serial.print(voltage1, 2);
            Serial.print("V, PA5: ");
            Serial.print(voltage2, 2);
            Serial.println("V");
        }
    }
    
    if (manualMode) {
        // Membaca nilai analog (0 - 4095)
        int val1 = analogRead(ANALOG1);
        int val2 = analogRead(ANALOG2);
        
        // Mengatur DAC berdasarkan nilai ADC
        analogWrite(DAC1, val1 > 0 ? val1 : IDLE_STATE);
        analogWrite(DAC2, val2 > 0 ? val2 : IDLE_STATE);
    }
    
    delay(10);
}