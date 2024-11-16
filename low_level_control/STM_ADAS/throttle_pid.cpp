#include <Arduino.h>

// Pin Configuration
const int THROTTLE_OUT1 = D5;
const int THROTTLE_OUT2 = D6;

// Throttle Variables
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 2V
const float maxOut2 = 1.0;   // 1V
const float dacMaxVoltage = 3.3;
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// PID Variables
float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain
float error_sum = 0.0;
float last_error = 0.0;
float desired_speed = 0.0;
unsigned long last_time = 0;

// Timing
const unsigned long SAMPLE_TIME_MS = 20;  // 20ms (50Hz) sampling rate

void setVoltage(int dacPin, float voltage) {
  int dacValue = (voltage / dacMaxVoltage) * 255;
  dacValue = constrain(dacValue, 0, 255);
  analogWrite(dacPin, dacValue);
}

void setThrottle(float throttle_percent) {
  // Map throttle percentage (0-100) to voltage range
  float normalized = throttle_percent / 100.0;
  currentOut2 = idleOut2 + normalized * (maxOut2 - idleOut2);
  currentOut1 = currentOut2 * 2;  // Out1 is always 2 times Out2
  
  currentOut1 = constrain(currentOut1, idleOut1, maxOut1);
  currentOut2 = constrain(currentOut2, idleOut2, maxOut2);
  
  setVoltage(THROTTLE_OUT1, currentOut1);
  setVoltage(THROTTLE_OUT2, currentOut2);
}

void setup() {
  Serial.begin(9600);  // Changed to 9600 baud rate for Nucleo F446RE
  
  // Initialize throttle outputs
  setVoltage(THROTTLE_OUT1, currentOut1);
  setVoltage(THROTTLE_OUT2, currentOut2);
  
  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  
  // Only process if sample time has elapsed
  if (current_time - last_time >= SAMPLE_TIME_MS) {
    if (Serial.available() >= sizeof(float)) {
      // Read desired speed from Python
      Serial.readBytes((char*)&desired_speed, sizeof(float));
      
      // Send acknowledgment
      Serial.write('A');
    }
    
    // Read current speed (assuming it's sent as a float from Python)
    if (Serial.available() >= sizeof(float)) {
      float current_speed;
      Serial.readBytes((char*)&current_speed, sizeof(float));
      
      // Calculate PID
      float dt = SAMPLE_TIME_MS / 1000.0;  // Convert to seconds
      
      float error = desired_speed - current_speed;
      error_sum += error * dt;
      float error_rate = (error - last_error) / dt;
      
      // Calculate control output (throttle percentage)
      float control = Kp * error + Ki * error_sum + Kd * error_rate;
      control = constrain(control, 0, 100);  // Limit to 0-100%
      
      // Apply control to throttle
      setThrottle(control);
      
      // Update variables for next iteration
      last_error = error;
      
      // Send control value back to Python for logging
      Serial.write((char*)&control, sizeof(float));
    }
    
    last_time = current_time;
  }
}