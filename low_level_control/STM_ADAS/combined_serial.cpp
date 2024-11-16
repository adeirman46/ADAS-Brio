#include <Arduino.h>
#include <Servo.h>

// Pin Configuration
const int THROTTLE_OUT1 = D5;  // DAC1 (default resolution)
const int THROTTLE_OUT2 = D6;  // DAC2 (default resolution)
const int BRAKE_SERVO = D7;    // Servo for brake control
const int EPS_RPWM = D3;       // Motor control forward PWM
const int EPS_LPWM = D11;      // Motor control reverse PWM
const int EPS_R_EN = D8;      // Motor enable pin
const int EPS_L_EN = D4;      // Motor enable pin

// Throttle Variables
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 2V
const float maxOut2 = 1.0;   // 1V
const float dacMaxVoltage = 3.3;
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// Brake Servo Variables
Servo brakeServo;
int brakeValue = 90;
const int brakeStep = 16;
const int MIN_ANGLE = 90;
const int MAX_ANGLE = 180;
const int ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;

// Motor Control Variables
int motorSpeed = 0;
const int speedIncrement = 20;

// Function to set the throttle voltage
void setVoltage(int dacPin, float voltage) {
  int dacValue = (voltage / dacMaxVoltage) * 255;  // Default 8-bit resolution (0-255)
  dacValue = constrain(dacValue, 0, 255);
  analogWrite(dacPin, dacValue);
}

// Initialize all components
void setup() {
  Serial.begin(9600);

  // Initialize Brake Servo
  brakeServo.attach(BRAKE_SERVO);
  brakeServo.write(MIN_ANGLE);

  // Initialize Motor Control (EPS)
  pinMode(EPS_RPWM, OUTPUT);
  pinMode(EPS_LPWM, OUTPUT);
  pinMode(EPS_R_EN, OUTPUT);
  pinMode(EPS_L_EN, OUTPUT);
  digitalWrite(EPS_R_EN, HIGH);
  digitalWrite(EPS_L_EN, HIGH);
  analogWrite(EPS_RPWM, 0);
  analogWrite(EPS_LPWM, 0);

  // Set initial idle state for throttle
  setVoltage(THROTTLE_OUT1, currentOut1);
  setVoltage(THROTTLE_OUT2, currentOut2);

  Serial.println("System initialized. Use commands:");
  Serial.println("Servo: 'b' increase brake, 'n' decrease brake");
  Serial.println("Motor: 'd' increase speed, 'a' decrease speed");
  Serial.println("Throttle: 'w' increase voltage, 's' decrease voltage");
}

void setMotorSpeed(int16_t speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    analogWrite(EPS_LPWM, 0);
    analogWrite(EPS_RPWM, speed);
  } else {
    analogWrite(EPS_RPWM, 0);
    analogWrite(EPS_LPWM, -speed);
  }
  Serial.print("Motor speed set to: ");
  Serial.println(speed);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Servo Control (Brake)
    if (command == 'b') {
      if (brakeValue <= (MAX_ANGLE - brakeStep)) {
        brakeValue += brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print(((brakeValue - MIN_ANGLE) * 100) / ANGLE_RANGE);
        Serial.println("%");
      }
    } else if (command == 'n') {
      if (brakeValue >= (MIN_ANGLE + brakeStep)) {
        brakeValue -= brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print(((brakeValue - MIN_ANGLE) * 100) / ANGLE_RANGE);
        Serial.println("%");
      }
    }
    
    // Motor Control (EPS)
    else if (command == 'd') {
      motorSpeed += speedIncrement;
      motorSpeed = constrain(motorSpeed, -255, 255);
      setMotorSpeed(motorSpeed);
      Serial.print("Motor speed: ");
      Serial.println(motorSpeed);
    } else if (command == 'a') {
      motorSpeed -= speedIncrement;
      motorSpeed = constrain(motorSpeed, -255, 255);
      setMotorSpeed(motorSpeed);
      Serial.print("Motor speed: ");
      Serial.println(motorSpeed);
    }

    // Throttle Control
    else if (command == 'w') {
      if (currentOut2 < maxOut2) {
        currentOut2 += 0.1;
        currentOut1 = currentOut2 * 2; // Out1 is always 2 times Out2
        if (currentOut1 > maxOut1) {
          currentOut1 = maxOut1;
        }
        
        setVoltage(THROTTLE_OUT1, currentOut1);
        setVoltage(THROTTLE_OUT2, currentOut2);
        
        Serial.print("Increased - Out1: ");
        Serial.print(currentOut1);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2);
        Serial.println("V");
      } else {
        Serial.println("Maximum voltage reached");
      }
    } else if (command == 's') {
      if (currentOut2 > idleOut2) {
        currentOut2 -= 0.1;
        currentOut1 = currentOut2 * 2; // Out1 is always 2 times Out2
        if (currentOut1 < idleOut1) {
          currentOut1 = idleOut1;
        }
        
        setVoltage(THROTTLE_OUT1, currentOut1);
        setVoltage(THROTTLE_OUT2, currentOut2);
        
        Serial.print("Decreased - Out1: ");
        Serial.print(currentOut1);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2);
        Serial.println("V");
      } else {
        Serial.println("Minimum voltage reached");
      }
    }
  }
  
  delay(100);  // Small delay for smoother control response
}