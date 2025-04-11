#include <Arduino.h>

const int POTENTIOMETER_PIN = A2;     // Define pin constant
const int SAMPLING_PERIOD = 20;       // 20ms = 50Hz sampling rate
unsigned long lastSampleTime = 0;

// ADC Constants
const int ADC_MAX = 1023;            // 10-bit ADC max value
const int ADC_CENTER = ADC_MAX / 2;  // Center position (511 for 10-bit)

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    pinMode(POTENTIOMETER_PIN, INPUT);
    Serial.println("Steering Angle Reader Started");
    Serial.println("Format: Raw ADC | Steering Angle (%)");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Only sample at 50Hz
    if (currentTime - lastSampleTime >= SAMPLING_PERIOD) {
        lastSampleTime = currentTime;
        
        // Read potentiometer with averaging for noise reduction
        int rawValue = 0;
        for(int i = 0; i < 3; i++) {
            rawValue += analogRead(POTENTIOMETER_PIN);
            delayMicroseconds(100);  // Short delay between samples
        }
        rawValue /= 3;  // Average of 3 readings
        
        // Calculate steering angle percentage (-200% to +200%)
        // Map 0-1023 to -200 to +200, with 511 as center
        float steeringAngle = ((float)(rawValue - ADC_CENTER) / ADC_CENTER) * 200.0;
        
        // Print values
        Serial.print("Raw: ");
        Serial.print(rawValue);
        Serial.print(" | Angle: ");
        Serial.print(steeringAngle, 1);  // Print with 1 decimal place
        Serial.println("%");
    }
}

