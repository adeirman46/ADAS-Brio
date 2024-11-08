#include <Arduino.h>

// Pins for IBT-2 module
#define R_PWM D5  // RPWM pin - Forward PWM
#define L_PWM D6  // LPWM pin - Reverse PWM
#define R_EN  D3  // R_EN pin
#define L_EN  D4  // L_EN pin

int motorSpeed = 0;          // Current motor speed, from -255 to 255
const int speedIncrement = 20; // Amount to increase/decrease the speed

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
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
  Serial.println("Press 'd' to increase speed, 'a' to decrease speed:");
}

void setMotorSpeed(int16_t speed) {
  // Constrain speed value to -255 to 255
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
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'd') {
      // Increase speed
      motorSpeed += speedIncrement;
      motorSpeed = constrain(motorSpeed, -255, 255); // Ensure within range
      setMotorSpeed(motorSpeed);
    } 
    else if (input == 'a') {
      // Decrease speed
      motorSpeed -= speedIncrement;
      motorSpeed = constrain(motorSpeed, -255, 255); // Ensure within range
      setMotorSpeed(motorSpeed);
    }
    
    // Print feedback to serial monitor
    Serial.print("Received input: ");
    Serial.print(input);
    Serial.print(" | Motor speed set to: ");
    Serial.println(motorSpeed);
  }
}
