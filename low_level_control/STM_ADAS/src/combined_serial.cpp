#include <Arduino.h>
#include <Servo.h>

// Pin Definitions - Using DAC capable pins on Nucleo F446RE
const int THROTTLE_OUT1 = D5;  // DAC1
const int THROTTLE_OUT2 = D6;  // DAC2
const int BRAKE_SERVO = D3;    
const int EPS_RPWM = D9;       
const int EPS_LPWM = D10;     
const int EPS_R_EN = D12;     
const int EPS_L_EN = D13;   

// Throttle constants
const float idleOut1 = 1.0;      // 1V
const float idleOut2 = 0.5;      // 0.5V
const float maxOut1 = 2.0;       // 2V
const float maxOut2 = 1.0;       // 1V
const float dacMaxVoltage = 3.3; // STM32 voltage reference
const int dacResolution = 4095;  // 12-bit resolution

// Brake constants
const int BRAKE_MIN = 90;        // Fully released position
const int BRAKE_MAX = 180;       // Fully pressed position
const int BRAKE_STEP = 8;        // Increment step

// EPS constants
const int EPS_STEP = 20;         // Speed increment
const int EPS_MAX_SPEED = 255;   // Maximum motor speed

// Global variables
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;
int brakePosition = BRAKE_MIN;
int epsSpeed = 0;
Servo brakeServo;
bool throttleEnabled = false;
unsigned long startTime;

// Helper function to set throttle voltage
void setThrottleVoltage(int pin, float voltage) {
  int dacValue = (voltage / dacMaxVoltage) * dacResolution;
  dacValue = constrain(dacValue, 0, dacResolution);
  analogWrite(pin, dacValue);
}

// Helper function to set EPS motor speed
void setEPSSpeed(int16_t speed) {
  speed = constrain(speed, -EPS_MAX_SPEED, EPS_MAX_SPEED);
  
  if (speed >= 0) {
    analogWrite(EPS_LPWM, 0);
    analogWrite(EPS_RPWM, speed);
  } else {
    analogWrite(EPS_RPWM, 0);
    analogWrite(EPS_LPWM, -speed);
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure pins
  analogWriteResolution(12); // Set 12-bit resolution for DAC
  
  // Initialize throttle to idle state
  setThrottleVoltage(THROTTLE_OUT1, currentOut1);
  setThrottleVoltage(THROTTLE_OUT2, currentOut2);
  
  // Initialize brake servo
  brakeServo.attach(BRAKE_SERVO);
  brakeServo.write(BRAKE_MIN);
  
  // Initialize EPS
  pinMode(EPS_RPWM, OUTPUT);
  pinMode(EPS_LPWM, OUTPUT);
  pinMode(EPS_R_EN, OUTPUT);
  pinMode(EPS_L_EN, OUTPUT);
  digitalWrite(EPS_R_EN, HIGH);
  digitalWrite(EPS_L_EN, HIGH);
  
  Serial.println("System initializing...");
  Serial.println("Holding idle state for 15 seconds...");
  Serial.println("Current throttle values:");
  Serial.printf("Out1: %.2fV, Out2: %.2fV\n", currentOut1, currentOut2);
  
  // Record start time
  startTime = millis();
}

void loop() {
  // Check if 15 seconds have passed
  if (!throttleEnabled && (millis() - startTime) >= 15000) {
    throttleEnabled = true;
    Serial.println("\nHold period complete!");
    Serial.println("Controls now active:");
    Serial.println("W/S - Throttle increase/decrease");
    Serial.println("A/D - EPS left/right");
    Serial.println("B/N - Brake increase/decrease");
  }

  // During hold period, only show countdown
  if (!throttleEnabled) {
    static unsigned long lastPrint = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrint >= 1000) { // Print every second
      int remainingSeconds = 15 - ((currentTime - startTime) / 1000);
      Serial.printf("Remaining hold time: %d seconds\n", remainingSeconds);
      lastPrint = currentTime;
    }
    return; // Skip the rest of the loop during hold period
  }

  // Normal operation after hold period
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case 'w': // Increase throttle
        if (currentOut2 < maxOut2) {
          currentOut2 += 0.1;
          currentOut1 = currentOut2 * 2;
          if (currentOut1 > maxOut1) currentOut1 = maxOut1;
          
          setThrottleVoltage(THROTTLE_OUT1, currentOut1);
          setThrottleVoltage(THROTTLE_OUT2, currentOut2);
          
          Serial.printf("Throttle - Out1: %.2fV, Out2: %.2fV\n", currentOut1, currentOut2);
        }
        break;
        
      case 's': // Decrease throttle
        if (currentOut2 > idleOut2) {
          currentOut2 -= 0.1;
          currentOut1 = currentOut2 * 2;
          if (currentOut1 < idleOut1) currentOut1 = idleOut1;
          
          setThrottleVoltage(THROTTLE_OUT1, currentOut1);
          setThrottleVoltage(THROTTLE_OUT2, currentOut2);
          
          Serial.printf("Throttle - Out1: %.2fV, Out2: %.2fV\n", currentOut1, currentOut2);
        }
        break;
        
      case 'a': // EPS left
        epsSpeed = constrain(epsSpeed - EPS_STEP, -EPS_MAX_SPEED, EPS_MAX_SPEED);
        setEPSSpeed(epsSpeed);
        Serial.printf("EPS Speed: %d\n", epsSpeed);
        break;
        
      case 'd': // EPS right
        epsSpeed = constrain(epsSpeed + EPS_STEP, -EPS_MAX_SPEED, EPS_MAX_SPEED);
        setEPSSpeed(epsSpeed);
        Serial.printf("EPS Speed: %d\n", epsSpeed);
        break;
        
      case 'b': // Increase brake
        if (brakePosition < BRAKE_MAX) {
          brakePosition += BRAKE_STEP;
          brakeServo.write(brakePosition);
          Serial.printf("Brake Position: %d%%\n", 
                       ((brakePosition - BRAKE_MIN) * 100) / (BRAKE_MAX - BRAKE_MIN));
        }
        break;
        
      case 'n': // Decrease brake
        if (brakePosition > BRAKE_MIN) {
          brakePosition -= BRAKE_STEP;
          brakeServo.write(brakePosition);
          Serial.printf("Brake Position: %d%%\n", 
                       ((brakePosition - BRAKE_MIN) * 100) / (BRAKE_MAX - BRAKE_MIN));
        }
        break;
    }
  }
  
  delay(10); // Reduced delay for smoother control response
}
