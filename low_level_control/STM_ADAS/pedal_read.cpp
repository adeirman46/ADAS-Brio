#include <Arduino.h>

const int analog_pin = PA7; 

void setup(){
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    pinMode(analog_pin, INPUT);
}

void loop(){
    // Read the analog value
    int rawValue = analogRead(analog_pin);
    
    // Map the raw value (140 to 750) to percentage (0 to 100)
    float percentage = map(rawValue, 140, 750, 0, 100);
    
    // Constrain the percentage to ensure it stays between 0 and 100
    percentage = constrain(percentage, 0, 100);
    
    // Get the current time in milliseconds
    unsigned long time_ms = millis();
    
    // Send data in CSV format: time_ms,raw_value,percentage
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.print(percentage);
    Serial.println();
    
    // Add a small delay to avoid flooding the serial output
    delay(100);  // Adjust as needed
}