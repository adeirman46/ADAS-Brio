#include <Arduino.h>

const int POTENTIOMETER_PIN = PB0;     // Define pin constant
// ADC Constants
const int ADC_MAX = 1023;            // 10-bit ADC max value
const int ADC_CENTER = ADC_MAX / 2;  // Center position (511 for 10-bit)

// FIR Filter coefficients (20th order)
const float b_coeffs[] = {-0.0019, -0.0017, 0.0065, 0.0003, -0.0207, 0.0144, 0.0430, -0.0668, -0.0641, 0.3040, 0.5742, 0.3040, -0.0641, -0.0668, 0.0430, 0.0144, -0.0207, 0.0003, 0.0065, -0.0017, -0.0019};
const int b_len = sizeof(b_coeffs) / sizeof(b_coeffs[0]);

// Buffer for previous samples (size matches number of coefficients)
float prev_samples[21] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float filtered_value = 0.0;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    pinMode(POTENTIOMETER_PIN, INPUT);
    
    // Initialize buffer with first reading
    prev_samples[0] = analogRead(POTENTIOMETER_PIN);
    filtered_value = prev_samples[0];
}

void loop() {
    // Read potentiometer
    int rawValue = analogRead(POTENTIOMETER_PIN);
    
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
    
    // Calculate steering angle percentage (-200% to +200%) using filtered value
    float steeringAngle = ((float)(filtered_value - ADC_CENTER) / ADC_CENTER) * 200.0;
    
    // Get the current time in milliseconds
    unsigned long time_ms = millis();
    
    // Send data in CSV format: time_ms,raw_value,filtered_value,steering_angle
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.print(filtered_value);
    Serial.print(",");
    Serial.print(steeringAngle, 1);  // Print with 1 decimal place
    Serial.println();
    
    // Add a small delay to avoid flooding the serial output
    delay(100);  // Adjust as needed
}