#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Define constants for brake pedal position
const float BRAKE_RELEASED_MM = 114.0; // 11.4cm when pedal not pressed (0%)
const float BRAKE_FULL_MM = 52.0;      // 5.2cm when pedal fully pressed (100%)
const float BRAKE_RANGE_MM = BRAKE_RELEASED_MM - BRAKE_FULL_MM; // Travel range

// IIR Filter parameters
const float Wc = 0.3; // Cutoff frequency in rad/s
const float Ts = 0.1; // Sampling period in seconds (based on delay(100))

// Dynamically computed IIR filter coefficients
float b0, b1, b2, a1, a2;

// Buffers for previous inputs and outputs
float prev_input_1 = 0.0;
float prev_input_2 = 0.0;
float filtered_distance = 0.0;
float prev_filtered_1 = 0.0;
float prev_filtered_2 = 0.0;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(1);
    }

    Serial.println("Brake Pedal Depth Sensor System");
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X sensor"));
        while(1); // Halt if sensor initialization fails
    }
    
    Serial.println(F("VL53L0X brake pedal feedback system initialized"));
    
    // Start continuous ranging
    lox.startRangeContinuous();
    
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
    if (lox.isRangeComplete()) {
        float initial_value = lox.readRange();
        prev_input_1 = initial_value;
        prev_input_2 = initial_value;
        filtered_distance = initial_value;
        prev_filtered_1 = filtered_distance;
        prev_filtered_2 = filtered_distance;
    }
}

// Function to calculate brake pedal position as a percentage
float calculateBrakePercentage(float distance_mm) {
    // Constrain the value to be within the defined range
    if (distance_mm > BRAKE_RELEASED_MM) {
        distance_mm = BRAKE_RELEASED_MM; // Cap at maximum distance (0% pressed)
    } else if (distance_mm < BRAKE_FULL_MM) {
        distance_mm = BRAKE_FULL_MM;     // Cap at minimum distance (100% pressed)
    }
    
    // Calculate percentage: 0% when released, 100% when fully pressed
    float percentage = ((BRAKE_RELEASED_MM - distance_mm) / BRAKE_RANGE_MM) * 100.0;
    
    return percentage;
}

void loop() {
    if (lox.isRangeComplete()) {
        float raw_distance = lox.readRange();
        
        // Apply 2nd-order IIR filter
        filtered_distance = b0 * raw_distance + b1 * prev_input_1 + b2 * prev_input_2 - a1 * prev_filtered_1 - a2 * prev_filtered_2;
        
        // Update previous values
        prev_input_2 = prev_input_1;
        prev_input_1 = raw_distance;
        prev_filtered_2 = prev_filtered_1;
        prev_filtered_1 = filtered_distance;
        
        float brake_percent = calculateBrakePercentage(filtered_distance);
        
        // Log time (in milliseconds since start), raw distance, filtered distance, and brake percentage as CSV format
        unsigned long time_ms = millis();
        Serial.print(time_ms);
        Serial.print(",");
        Serial.print(raw_distance);
        Serial.print(",");
        Serial.print(filtered_distance);
        Serial.print(",");
        Serial.println(brake_percent);
        delay(100); // Delay for readability
    }
}