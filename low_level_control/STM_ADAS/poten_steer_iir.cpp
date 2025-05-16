#include <Arduino.h>

const int POTENTIOMETER_PIN = PB0;     // Define pin constant
// ADC Constants
const int ADC_MAX = 1023;            // 10-bit ADC max value
const int ADC_CENTER = 830; //ADC_MAX / 2;  // Center position (511 for 10-bit)

// IIR Filter parameters
const float Wc = 2.8629; // Cutoff frequency in rad/s
const float Ts = 0.1;    // Sampling period in seconds (based on delay(100))

// Dynamically computed IIR filter coefficients
float b0, b1, b2, a1, a2;

// Buffers for previous inputs and outputs (moved to global scope)
float prev_input_1 = 0.0;
float prev_input_2 = 0.0;
float filtered_value = 0.0;
float prev_filtered_1 = 0.0;
float prev_filtered_2 = 0.0;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    pinMode(POTENTIOMETER_PIN, INPUT);
    
    // Calculate alpha
    float alpha = (Wc * Ts) / 2.0;
    
    // Calculate denominator factor
    float denom = 1.0 + sqrt(2.0) * alpha + alpha * alpha;
    
    // Compute coefficients
    a1 = -2.0 * (1.0 - alpha * alpha) / denom;
    a2 = (1.0 - sqrt(2.0) * alpha + alpha * alpha) / denom;
    b0 = (alpha * alpha) / denom;
    b1 = 2.0 * b0;
    b2 = b0;
    
    // Initialize filter with first reading
    float initial_value = analogRead(POTENTIOMETER_PIN);
    prev_input_1 = initial_value;
    prev_input_2 = initial_value;
    filtered_value = initial_value;
    prev_filtered_1 = filtered_value;
    prev_filtered_2 = filtered_value;
}

void loop() {
    // Read potentiometer
    float rawValue = analogRead(POTENTIOMETER_PIN);
    
    // Apply 2nd-order IIR filter
    filtered_value = b0 * rawValue + b1 * prev_input_1 + b2 * prev_input_2 - a1 * prev_filtered_1 - a2 * prev_filtered_2;
    
    // Update previous values
    prev_input_2 = prev_input_1;
    prev_input_1 = rawValue;
    prev_filtered_2 = prev_filtered_1;
    prev_filtered_1 = filtered_value;
    
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