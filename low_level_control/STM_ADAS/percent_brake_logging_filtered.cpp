#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Define constants for brake pedal position
const float BRAKE_RELEASED_MM = 114.0; // 11.4cm when pedal not pressed (0%)
const float BRAKE_FULL_MM = 52.0;      // 5.2cm when pedal fully pressed (100%)
const float BRAKE_RANGE_MM = BRAKE_RELEASED_MM - BRAKE_FULL_MM; // Travel range

// IIR Filter parameters
const float wc = 0.3; // Cutoff frequency in Hz
const float RC = 1.0 / wc; // Time constant (approx. 3.333 seconds)
const float Ts = 0.1; // Sampling period in seconds (based on delay(100))
const float a = RC / Ts; // Filter coefficient (approx. 33.33)
float filtered_distance = 0.0;
float prev_filtered_distance = 0.0;

void setup() {
  Serial.begin(9600);

  // Wait until serial port opens for native USB devices
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
  
  // Initialize filtered distance with the first reading
  if (lox.isRangeComplete()) {
    prev_filtered_distance = lox.readRange();
    filtered_distance = prev_filtered_distance;
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
    float distance_mm = lox.readRange();
    
    // Apply IIR filter
    filtered_distance = (distance_mm + a * prev_filtered_distance) / (a + 1);
    prev_filtered_distance = filtered_distance;
    
    float brake_percent = calculateBrakePercentage(filtered_distance);
    
    // Log time (in milliseconds since start), filtered distance, and brake percentage as CSV format
    unsigned long currentTime = millis();  // Get the current time in milliseconds
    Serial.print(currentTime);
    Serial.print(", ");
    Serial.print(filtered_distance);
    Serial.print(", ");
    Serial.println(brake_percent);
    
    delay(100); // Delay for readability
  }
}