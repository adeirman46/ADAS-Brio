#include <Arduino.h>
#include <Servo.h>

// Pin Configuration
const int THROTTLE_OUT1 = PB10;  // DAC1 (default resolution)
const int THROTTLE_OUT2 = PB4;  // DAC2 (default resolution)
const int BRAKE_SERVO = PA9;    // Servo for brake control

// Steering Configuration
const int STEER_OUT1 = PA4;    // Steering DAC output 1
const int STEER_OUT2 = PA5;    // Steering DAC output 2

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
int brakeValue = 0;
const int brakeStep = 18;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;

// Steering Variables
float steerVoltage1 = 2.5;  // Center position (2.5V)
float steerVoltage2 = 2.5;  // Center position (2.5V)
const float STEER_CENTER = 2.5;
const float STEER_MAX_DELTA = 0.8;  // Maximum voltage change from center
const int IDLE_STATE = 3100;         // 2.5V on 12-bit scale (2.5/3.3*4095)

// Function to set the throttle voltage
void setVoltage(int dacPin, float voltage) {
  int dacValue = (voltage / dacMaxVoltage) * 255;  // Default 8-bit resolution (0-255)
  dacValue = constrain(dacValue, 0, 255);
  analogWrite(dacPin, dacValue);
}

// Function to set steering voltage with 12-bit resolution
void setSteeringVoltage(int dacPin, float voltage) {
  int dacValue = (voltage / 3.3) * 4095;  // 12-bit resolution (0-4095)
  dacValue = constrain(dacValue, 0, 4095);
  analogWrite(dacPin, dacValue);
}

// Initialize all components
void setup() {
  Serial.begin(9600);

  // Initialize Brake Servo
  brakeServo.attach(BRAKE_SERVO);
  brakeServo.write(MIN_ANGLE);

  // Set analog and DAC resolution for steering
  analogReadResolution(12);  // 12-bit resolution for reading torque sensor
  analogWriteResolution(12); // 12-bit resolution for steering output

  // Set initial idle state for throttle
  setVoltage(THROTTLE_OUT1, currentOut1);
  setVoltage(THROTTLE_OUT2, currentOut2);

  // Set initial center position for steering
  setSteeringVoltage(STEER_OUT1, STEER_CENTER);
  setSteeringVoltage(STEER_OUT2, STEER_CENTER);

  Serial.println("System initialized. Use commands:");
  Serial.println("Servo: 'b' increase brake, 'n' decrease brake");
  Serial.println("Throttle: 'w' increase voltage, 's' decrease voltage");
  Serial.println("Steering: 'a' steer left, 'd' steer right");
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
        Serial.print((brakeValue * 100) / ANGLE_RANGE);  // Calculate percentage
        Serial.println("%");
      }
    } else if (command == 'n') {
      if (brakeValue >= (MIN_ANGLE + brakeStep)) {
        brakeValue -= brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print((brakeValue * 100) / ANGLE_RANGE);  // Calculate percentage
        Serial.println("%");
      }
    }
    
    // Steering Control
    else if (command == 'a' || command == 'd') {
      if (command == 'a') {  // Steer left
        if (steerVoltage1 < (STEER_CENTER + STEER_MAX_DELTA) && 
            steerVoltage2 > (STEER_CENTER - STEER_MAX_DELTA)) {
          steerVoltage1 += 0.1;
          steerVoltage2 -= 0.1;
        }
      } else if (command == 'd') {  // Steer right
        if (steerVoltage1 > (STEER_CENTER - STEER_MAX_DELTA) && 
            steerVoltage2 < (STEER_CENTER + STEER_MAX_DELTA)) {
          steerVoltage1 -= 0.1;
          steerVoltage2 += 0.1;
        }
      }
      
      setSteeringVoltage(STEER_OUT1, steerVoltage1);
      setSteeringVoltage(STEER_OUT2, steerVoltage2);
      
      Serial.print("Steering - Out1: ");
      Serial.print(steerVoltage1, 2);
      Serial.print("V, Out2: ");
      Serial.print(steerVoltage2, 2);
      Serial.println("V");
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
  // No torque sensor processing needed
  
  delay(50);  // Small delay for smoother control response
}