#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Define constants for brake pedal position
const float BRAKE_RELEASED_MM = 114.0; // 11.4cm when pedal not pressed (0%)
const float BRAKE_FULL_MM = 52.0;      // 5.2cm when pedal fully pressed (100%)
const float BRAKE_RANGE_MM = BRAKE_RELEASED_MM - BRAKE_FULL_MM; // Travel range

// FIR Filter coefficients
const float b_coeffs[] = {0.0041,    0.0061,     0.0112,     0.0200,     0.0322,     0.0469,     0.0627,     0.0777,     0.0901,     0.0983,     0.1012,     0.0983,     0.0901,    0.0777,     0.0627,     0.0469,     0.0322,     0.0200,     0.0112,     0.0061,     0.0041}; // b0 to b10
const int b_len = sizeof(b_coeffs) / sizeof(b_coeffs[0]);

float filtered_distance = 0.0;
float prev_samples[21] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Buffer for previous samples (size matches number of coefficients)

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
  
  // Initialize with the first reading
  if (lox.isRangeComplete()) {
    prev_samples[0] = lox.readRange();
    filtered_distance = prev_samples[0];
  }
}

// Function to calculate brake pedal position as a percentage
float calculateBrakePercentage(float distance_mm) {
  if (distance_mm > BRAKE_RELEASED_MM) {
    distance_mm = BRAKE_RELEASED_MM;
  } else if (distance_mm < BRAKE_FULL_MM) {
    distance_mm = BRAKE_FULL_MM;
  }
  
  float percentage = ((BRAKE_RELEASED_MM - distance_mm) / BRAKE_RANGE_MM) * 100.0;
  return percentage;
}

void loop() {
  if (lox.isRangeComplete()) {
    float distance_mm = lox.readRange();
    
    // Shift previous samples
    for (int i = b_len - 1; i > 0; i--) {
      prev_samples[i] = prev_samples[i - 1];
    }
    prev_samples[0] = distance_mm;
    
    // Apply FIR filter (convolution)
    filtered_distance = 0.0;
    for (int i = 0; i < b_len; i++) {
      filtered_distance += b_coeffs[i] * prev_samples[i];
    }
    
    float brake_percent = calculateBrakePercentage(filtered_distance);
    
    // Log time, raw distance, filtered distance, and brake percentage as CSV
    unsigned long currentTime = millis();
    Serial.print(currentTime);
    Serial.print(", ");
    Serial.print(distance_mm);
    Serial.print(", ");
    Serial.print(filtered_distance);
    Serial.print(", ");
    Serial.println(brake_percent);
    
    delay(100);
  }
}