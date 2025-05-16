#include <Arduino.h>

const int analog_pin = PA7; 

// FIR Filter coefficients (10th order)
const float b_coeffs[] = {   -0.0015,   -0.0034,    -0.0065,    -0.0088,    -0.0051,     0.0109,     0.0420,     0.0853,     0.1311,     0.1663,     0.1795,     0.1663,     0.1311,     0.0853,     0.0420,     0.0109,    -0.0051,    -0.0088,    -0.0065,    -0.0034,    -0.0015};
const int b_len = sizeof(b_coeffs) / sizeof(b_coeffs[0]);

// Buffer for previous samples (size matches number of coefficients)
float prev_samples[21] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};
float filtered_value = 0.0;

void setup(){
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    pinMode(analog_pin, INPUT);
    
    // Initialize buffer with first reading
    prev_samples[0] = analogRead(analog_pin);
    filtered_value = prev_samples[0];
}

void loop(){
    // Read the analog value
    int rawValue = analogRead(analog_pin);
    
    // Shift previous samples
    for (int i = b_len - 1; i > 0; i--) {
        prev_samples[i] = prev_samples[i - 1];
    }
    prev_samples[0] = rawValue;
    
    // Apply FIR filter (convolution)
    filtered_value = 0.0;
    for (int i = 0; i < b_len; i++) {
        filtered_value += b_coeffs[i] * prev_samples[i];
    }
    
    // Map the filtered value (140 to 750) to percentage (0 to 100)
    float percentage = map(filtered_value, 140, 750, 0, 100);
    
    // Constrain the percentage to ensure it stays between 0 and 100
    percentage = constrain(percentage, 0, 100);
    
    // Get the current time in milliseconds
    unsigned long time_ms = millis();
    
    // Send data in CSV format: time_ms,raw_value,filtered_value,percentage
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.print(filtered_value);
    Serial.print(",");
    Serial.println(percentage);
    
    // Add a small delay to avoid flooding the serial output
    delay(100);  // Adjust as needed
}