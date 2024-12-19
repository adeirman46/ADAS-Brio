#include <Arduino.h>

const int OUT1_PIN = D5;  // DAC1
const int OUT2_PIN = D6;  // DAC2
const float DAC_MAX_VOLTAGE = 3.3;
const int DAC_RESOLUTION = 4095;

// Helper function to set voltage
void setVoltage(int pin, float voltage) {
    int dacValue = (voltage / DAC_MAX_VOLTAGE) * DAC_RESOLUTION;
    dacValue = constrain(dacValue, 0, DAC_RESOLUTION);
    analogWrite(pin, dacValue);
}

void setup() {
    Serial.begin(9600);
    analogWriteResolution(12);  // Set 12-bit resolution
    
    // Initialize outputs to minimum voltage
    setVoltage(OUT1_PIN, 1.0);
    setVoltage(OUT2_PIN, 0.5);
}

void loop() {
    if (Serial.available() >= sizeof(float)) {
        // Read float value
        float voltage;
        Serial.readBytes((char*)&voltage, sizeof(float));
        
        // Constrain voltage to valid range
        voltage = constrain(voltage, 0.5, 1.0);
        
        // Set OUT2 to received voltage and OUT1 to double that value
        setVoltage(OUT2_PIN, voltage);
        setVoltage(OUT1_PIN, voltage * 2.0);
        
        // Send confirmation back
        Serial.write((char*)&voltage, sizeof(float));
    }
}