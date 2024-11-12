#include <Arduino.h>
#include <Servo.h>

// Pin Configuration for IBT-2 Motor Driver
#define EPS_RPWM D5    // RPWM pin - Forward PWM
#define EPS_LPWM D6    // LPWM pin - Reverse PWM
#define EPS_R_EN D3    // R_EN pin
#define EPS_L_EN D4    // L_EN pin

// Additional Pin Configuration
const int THROTTLE_OUT1 = PA_4;  // DAC1 (Use appropriate analog pins)
const int THROTTLE_OUT2 = PA_5;  // DAC2 (Use appropriate analog pins)
const int BRAKE_SERVO = PB_6;    // Servo for brake control (PWM capable pin)

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
const int brakeStep = 8;
const int MIN_ANGLE = 90;
const int MAX_ANGLE = 180;
const int ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;

// Motor Control Variables
int motorSpeed = 0;
const int speedIncrement = 20;

// Function to set the throttle voltage
void setVoltage(int dacPin, float voltage) {
  int dacValue = (voltage / dacMaxVoltage) * 255;
  dacValue = constrain(dacValue, 0, 255);
  analogWrite(dacPin, dacValue);
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

void setup() {
  Serial.begin(9600);

  // Initialize Motor Control (EPS)
  pinMode(EPS_RPWM, OUTPUT);
  pinMode(EPS_LPWM, OUTPUT);
  pinMode(EPS_R_EN, OUTPUT);
  pinMode(EPS_L_EN, OUTPUT);
  digitalWrite(EPS_R_EN, HIGH);
  digitalWrite(EPS_L_EN, HIGH);
  analogWrite(EPS_RPWM, 0);
  analogWrite(EPS_LPWM, 0);

  // Initialize Brake Servo
  brakeServo.attach(BRAKE_SERVO);
  brakeServo.write(MIN_ANGLE);

  // Set initial idle state for throttle
  setVoltage(THROTTLE_OUT1, currentOut1);
  setVoltage(THROTTLE_OUT2, currentOut2);

  Serial.println("System initialized. Use commands:");
  Serial.println("Motor: 'd' increase speed, 'a' decrease speed");
  Serial.println("Brake: 'b' increase brake, 'n' decrease brake");
  Serial.println("Throttle: 'w' increase voltage, 's' decrease voltage");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Motor Control (EPS)
    if (command == 'd') {
      motorSpeed += speedIncrement;
      motorSpeed = constrain(motorSpeed, -255, 255);
      setMotorSpeed(motorSpeed);
    } 
    else if (command == 'a') {
      motorSpeed -= speedIncrement;
      motorSpeed = constrain(motorSpeed, -255, 255);
      setMotorSpeed(motorSpeed);
    }
    
    // Brake Control
    else if (command == 'b') {
      if (brakeValue <= (MAX_ANGLE - brakeStep)) {
        brakeValue += brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print(((brakeValue - MIN_ANGLE) * 100) / ANGLE_RANGE);
        Serial.println("%");
      }
    } 
    else if (command == 'n') {
      if (brakeValue >= (MIN_ANGLE + brakeStep)) {
        brakeValue -= brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print(((brakeValue - MIN_ANGLE) * 100) / ANGLE_RANGE);
        Serial.println("%");
      }
    }
    
    // Throttle Control
    else if (command == 'w') {
      if (currentOut2 < maxOut2) {
        currentOut2 += 0.1;
        currentOut1 = currentOut2 * 2;
        if (currentOut1 > maxOut1) currentOut1 = maxOut1;
        setVoltage(THROTTLE_OUT1, currentOut1);
        setVoltage(THROTTLE_OUT2, currentOut2);
        Serial.print("Throttle - Out1: ");
        Serial.print(currentOut1);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2);
        Serial.println("V");
      }
    } 
    else if (command == 's') {
      if (currentOut2 > idleOut2) {
        currentOut2 -= 0.1;
        currentOut1 = currentOut2 * 2;
        if (currentOut1 < idleOut1) currentOut1 = idleOut1;
        setVoltage(THROTTLE_OUT1, currentOut1);
        setVoltage(THROTTLE_OUT2, currentOut2);
        Serial.print("Throttle - Out1: ");
        Serial.print(currentOut1);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2);
        Serial.println("V");
      }
    }
  }
  
  delay(50);  // Smaller delay for more responsive control
}