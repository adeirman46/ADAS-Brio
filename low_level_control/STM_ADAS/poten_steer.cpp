#include <Arduino.h>

const int POTENTIOMETER_PIN = PB0;     // Define pin constant
// ADC Constants
const int ADC_MAX = 1023;            // 10-bit ADC max value
const int ADC_CENTER = ADC_MAX / 2;  // Center position (511 for 10-bit)

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    pinMode(POTENTIOMETER_PIN, INPUT);
}

void loop() {
    // Read potentiometer
    int rawValue = analogRead(POTENTIOMETER_PIN);
    
    // Calculate steering angle percentage (-200% to +200%)
    float steeringAngle = ((float)(rawValue - ADC_CENTER) / ADC_CENTER) * 200.0;
    
    // Get the current time in milliseconds
    unsigned long time_ms = millis();
    
    // Send data in CSV format: time_ms,raw_value,steering_angle
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.print(steeringAngle, 1);  // Print with 1 decimal place
    Serial.println();
    
    // Add a small delay to avoid flooding the serial output
    delay(100);  // Adjust as needed
}