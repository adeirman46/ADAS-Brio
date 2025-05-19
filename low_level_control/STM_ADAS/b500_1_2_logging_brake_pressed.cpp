#include <Arduino.h>
#include <Servo.h>  
#include <Adafruit_VL53L0X.h>

Servo myServo;  
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Define constants for brake pedal position
const float BRAKE_RELEASED_MM = 100.0; // 11.4cm when pedal not pressed (0%)
const float BRAKE_FULL_MM = 20.0;      // 5.2cm when pedal fully pressed (100%)
const float BRAKE_RANGE_MM = BRAKE_RELEASED_MM - BRAKE_FULL_MM; // Travel range

// IIR Filter parameters
const float Wc = 0.3*(2*PI); // Cutoff frequency in rad/s
const float Ts = 0.1; // Sampling period in seconds (based on delay(100))

// Dynamically computed IIR filter coefficients
float b0, b1, b2, a1, a2;

// Buffers for previous inputs and outputs
float prev_input_1 = 0.0;
float prev_input_2 = 0.0;
float filtered_distance = 0.0;
float prev_filtered_1 = 0.0;
float prev_filtered_2 = 0.0;

const int MIN_ANGLE = 0;   // Fully released
const int MAX_ANGLE = 180; // Fully pressed
int brakeValue = 0;        // Start at fully released position (0 degrees)
String brakeMode = "RELEASED"; // Track brake mode

void setup() {
    Serial.begin(9600);       
    while (!Serial) {
        delay(1);
    }
    
    myServo.attach(PA9);      
    myServo.write(MIN_ANGLE);  // Start with brake released (0 degrees)
    brakeValue = MIN_ANGLE;
    brakeMode = "RELEASED";
    
    Serial.println("Brake Control and VL53L0X Sensor System Initialized");
    Serial.println("Brake set to 0% (0 degrees, RELEASED)");
    
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X sensor"));
        while(1); // Halt if sensor initialization fails
    }
    
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
    // Check for serial input
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'b') {
            myServo.write(MAX_ANGLE); // Fully press brake
            brakeValue = MAX_ANGLE;
            brakeMode = "FULL";
            Serial.println("Brake set to 100% (180 degrees, FULL)");
        } else if (command == 'n') {
            myServo.write(MIN_ANGLE); // Fully release brake
            brakeValue = MIN_ANGLE;
            brakeMode = "RELEASED";
            Serial.println("Brake set to 0% (0 degrees, RELEASED)");
        }
    }

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
        
        // Log data with mode
        unsigned long time_ms = millis();
        Serial.print(time_ms);
        Serial.print(",");
        Serial.print(raw_distance);
        Serial.print(",");
        Serial.print(filtered_distance);
        Serial.print(",");
        Serial.print(brake_percent);
        Serial.print(",");
        Serial.println(brakeMode);
        
        delay(100); // Delay for readability
    }
}