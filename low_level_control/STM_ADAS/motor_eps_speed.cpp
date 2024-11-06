// Motor Control using IBT-2 H-Bridge with STM32

#include <Arduino.h>

// Pins for IBT-2 module
#define R_PWM D5  // RPWM pin - Forward PWM
#define L_PWM D6  // LPWM pin - Reverse PWM
#define R_EN  D3  // R_EN pin
#define L_EN  D4  // L_EN pin

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Configure pins
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  
  // Enable IBT-2 module
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  
  // Initialize motor to stopped state
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
  
  Serial.println("Motor controller initialized");
}

void setMotorSpeed(int16_t speed) {
  // Constrain speed value
  speed = constrain(speed, -255, 255);
  
  // Determine direction and PWM values
  if (speed >= 0) {
    // Forward direction
    analogWrite(L_PWM, 0);          // Set reverse PWM to 0
    analogWrite(R_PWM, speed);      // Set forward PWM
  } else {
    // Reverse direction
    analogWrite(R_PWM, 0);          // Set forward PWM to 0
    analogWrite(L_PWM, -speed);     // Set reverse PWM
  }
  
  // Print debug information
  Serial.print("Speed set to: ");
  Serial.println(speed);
}

void loop() {

  // Example usage pattern
  // Forward acceleration test
  Serial.println("Testing forward acceleration...");
  
  // Start slow
  Serial.println("Starting at 25% speed forward");
  setMotorSpeed(64);  // Ramp to 25% speed over 1 second
  delay(2000);          // Maintain speed for 2 seconds
  
  // Medium speed
  Serial.println("Increasing to 50% speed forward");
  setMotorSpeed(128); // Ramp to 50% speed over 1 second
  delay(2000);          // Maintain speed for 2 seconds
  
  // Full speed
  Serial.println("Increasing to 100% speed forward");
  setMotorSpeed(255); // Ramp to full speed over 1 second
  delay(2000);          // Maintain speed for 2 seconds
  
  // Stop
  Serial.println("Stopping...");
  setMotorSpeed(0);   // Ramp down to stop over 2 seconds
  delay(2000);          // Stay stopped for 2 seconds
  
  // Reverse test
  Serial.println("Testing reverse acceleration...");
  
  // Start slow reverse
  Serial.println("Starting at 25% speed reverse");
  setMotorSpeed(-64);  // Ramp to 25% reverse speed over 1 second
  delay(2000);           // Maintain speed for 2 seconds
  
  // Medium reverse
  Serial.println("Increasing to 50% speed reverse");
  setMotorSpeed(-128); // Ramp to 50% reverse speed over 1 second
  delay(2000);           // Maintain speed for 2 seconds
  
  // Full reverse
  Serial.println("Increasing to 100% speed reverse");
  setMotorSpeed(-255); // Ramp to full reverse speed over 1 second
  delay(2000);           // Maintain speed for 2 seconds
  
  // Stop again
  Serial.println("Stopping...");
  setMotorSpeed(0);    // Ramp down to stop over 2 seconds
  delay(2000);           // Stay stopped for 2 seconds
}