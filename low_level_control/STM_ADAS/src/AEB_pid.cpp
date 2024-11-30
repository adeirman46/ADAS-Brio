// #include <Arduino.h>
// #include <Servo.h>

// // Pin Configuration
// const int THROTTLE_OUT1 = D5;
// const int THROTTLE_OUT2 = D6;
// const int BRAKE_SERVO = D7;

// // Throttle Variables
// const float idleOut1 = 1.0;  // 1V
// const float idleOut2 = 0.5;  // 0.5V
// const float maxOut1 = 2.0;   // 2V
// const float maxOut2 = 1.0;   // 1V
// const float dacMaxVoltage = 3.3;
// float currentOut1 = idleOut1;
// float currentOut2 = idleOut2;

// // Brake Variables
// Servo brakeServo;
// const int BRAKE_RELEASED = 90;    // Angle for fully released brake
// const int BRAKE_PRESSED = 180;    // Angle for fully pressed brake
// int currentBrakeAngle = BRAKE_RELEASED;

// // Control Variables
// float desired_speed = 0.0;
// float current_speed = 0.0;
// float error = 0.0;
// float last_error = 0.0;
// float error_sum = 0.0;

// // Controller gains (separate for throttle and brake)
// struct ControllerGains {
//     float Kp;
//     float Ki;
//     float Kd;
// };

// // Define gains for throttle and brake
// ControllerGains throttleGains = {1.3918, 3.9967, 0.0};  // Tuned PID gains
// ControllerGains brakeGains = {5.0, 1.0, 0.0};          // Tuned PID gains

// // Timing
// const unsigned long SAMPLE_TIME_MS = 20;  // 50Hz sampling rate
// unsigned long last_time = 0;

// // Dead zone for speed error where neither throttle nor brake is applied
// const float SPEED_DEADZONE = 0.5;  // ±0.5 kph dead zone

// // Function to set voltage for throttle control
// void setVoltage(int dacPin, float voltage) {
//     int dacValue = (voltage / dacMaxVoltage) * 255;
//     dacValue = constrain(dacValue, 0, 255);
//     analogWrite(dacPin, dacValue);
// }

// // Function to set throttle position (0-100%)
// void setThrottle(float throttle_percent) {
//     float normalized = throttle_percent / 100.0;
//     currentOut2 = idleOut2 + normalized * (maxOut2 - idleOut2);
//     currentOut1 = currentOut2 * 2;  // Out1 is always 2 times Out2
    
//     currentOut1 = constrain(currentOut1, idleOut1, maxOut1);
//     currentOut2 = constrain(currentOut2, idleOut2, maxOut2);
    
//     setVoltage(THROTTLE_OUT1, currentOut1);
//     setVoltage(THROTTLE_OUT2, currentOut2);
// }

// // Function to set brake position (0-100%)
// void setBrake(float brake_percent) {
//     brake_percent = constrain(brake_percent, 0, 100);
//     int angle = map(brake_percent, 0, 100, BRAKE_RELEASED, BRAKE_PRESSED);
//     brakeServo.write(angle);
//     currentBrakeAngle = angle;
// }

// // Structure to hold control outputs
// struct ControlOutputs {
//     float throttle_percent;
//     float brake_percent;
// };

// // Function to calculate MIMO control outputs
// ControlOutputs calculateControl(float dt) {
//     ControlOutputs outputs = {0, 0};  // Initialize outputs to zero
    
//     // Calculate error
//     error = desired_speed - current_speed;
    
//     // Check if we're in the dead zone
//     if (abs(error) < SPEED_DEADZONE) {
//         error_sum = 0;  // Reset integral term
//         return outputs; // Return zero throttle and brake
//     }
    
//     // Calculate derivative and integral terms
//     float error_rate = (error - last_error) / dt;
//     error_sum += error * dt;
    
//     // Determine whether to use throttle or brake based on error sign
//     if (error > 0) {  // Need to speed up - use throttle
//         float control = throttleGains.Kp * error + 
//                        throttleGains.Ki * error_sum + 
//                        throttleGains.Kd * error_rate;
                       
//         outputs.throttle_percent = constrain(control, 0, 100);
//         outputs.brake_percent = 0;  // Ensure brake is released
//     }
//     else {  // Need to slow down - use brake
//         float control = -(brakeGains.Kp * error + 
//                          brakeGains.Ki * error_sum + 
//                          brakeGains.Kd * error_rate);
                         
//         outputs.throttle_percent = 0;  // Ensure throttle is closed
//         outputs.brake_percent = constrain(control, 0, 100);
//     }
    
//     last_error = error;  // Store error for next iteration
//     return outputs;
// }

// void setup() {
//     Serial.begin(9600);
    
//     // Initialize brake servo
//     brakeServo.attach(BRAKE_SERVO);
//     brakeServo.write(BRAKE_RELEASED);
    
//     // Initialize throttle outputs
//     setVoltage(THROTTLE_OUT1, currentOut1);
//     setVoltage(THROTTLE_OUT2, currentOut2);
    
//     last_time = millis();
// }

// void loop() {
//     unsigned long current_time = millis();
    
//     // Only process if sample time has elapsed
//     if (current_time - last_time >= SAMPLE_TIME_MS) {
//         float dt = SAMPLE_TIME_MS / 1000.0;  // Convert to seconds
        
//         // Read desired and current speed from Python
//         if (Serial.available() >= 2 * sizeof(float)) {
//             Serial.readBytes((char*)&desired_speed, sizeof(float));
//             Serial.readBytes((char*)&current_speed, sizeof(float));
            
//             // Calculate control outputs
//             ControlOutputs outputs = calculateControl(dt);
            
//             // Apply control outputs
//             setThrottle(outputs.throttle_percent);
//             setBrake(outputs.brake_percent);
            
//             // Send control outputs back to Python
//             Serial.write((char*)&outputs.throttle_percent, sizeof(float));
//             Serial.write((char*)&outputs.brake_percent, sizeof(float));
//         }
        
//         last_time = current_time;
//     }
// }

#include <Arduino.h>
#include <Servo.h>

// Pin Configuration
const int THROTTLE_OUT1 = D5;
const int THROTTLE_OUT2 = D6;
const int BRAKE_SERVO = D7;

// Throttle Variables
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 2V
const float maxOut2 = 1.0;   // 1V
const float dacMaxVoltage = 3.3;
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// Brake Variables
Servo brakeServo;
const int BRAKE_RELEASED = 90;    // Angle for fully released brake
const int BRAKE_PRESSED = 180;    // Angle for fully pressed brake
int currentBrakeAngle = BRAKE_RELEASED;

// Control Variables
float desired_speed = 0.0;
float current_speed = 0.0;
float error = 0.0;
float last_error = 0.0;
float error_sum = 0.0;

// PID gains for throttle only
float Kp = 1.3918;
float Ki = 3.9967;
float Kd = 0.0;

// Timing
const unsigned long SAMPLE_TIME_MS = 20;  // 50Hz sampling rate
unsigned long last_time = 0;

// Function to set voltage for throttle control
void setVoltage(int dacPin, float voltage) {
    int dacValue = (voltage / dacMaxVoltage) * 255;
    dacValue = constrain(dacValue, 0, 255);
    analogWrite(dacPin, dacValue);
}

// Function to set throttle position (0-100%)
void setThrottle(float throttle_percent) {
    float normalized = throttle_percent / 100.0;
    currentOut2 = idleOut2 + normalized * (maxOut2 - idleOut2);
    currentOut1 = currentOut2 * 2;  // Out1 is always 2 times Out2
    
    currentOut1 = constrain(currentOut1, idleOut1, maxOut1);
    currentOut2 = constrain(currentOut2, idleOut2, maxOut2);
    
    setVoltage(THROTTLE_OUT1, currentOut1);
    setVoltage(THROTTLE_OUT2, currentOut2);
}

// Function to set brake position (0-100%)
void setBrake(float brake_percent) {
    brake_percent = constrain(brake_percent, 0, 100);
    int angle = map(brake_percent, 0, 100, BRAKE_RELEASED, BRAKE_PRESSED);
    brakeServo.write(angle);
    currentBrakeAngle = angle;
}

// Function to calculate throttle using PID
float calculateThrottle(float dt) {
    // Calculate error
    error = desired_speed - current_speed;
    
    // Only use PID for throttle when we need to speed up
    if (error <= 0) {
        error_sum = 0;  // Reset integral term
        return 0;  // No throttle needed
    }
    
    // Calculate derivative and integral terms
    float error_rate = (error - last_error) / dt;
    error_sum += error * dt;
    
    // Calculate PID output for throttle
    float control = Kp * error + Ki * error_sum + Kd * error_rate;
    
    last_error = error;  // Store error for next iteration
    return constrain(control, 0, 100);  // Constrain to valid throttle range
}

void setup() {
    Serial.begin(9600);
    
    // Initialize brake servo
    brakeServo.attach(BRAKE_SERVO);
    brakeServo.write(BRAKE_RELEASED);
    
    // Initialize throttle outputs
    setVoltage(THROTTLE_OUT1, currentOut1);
    setVoltage(THROTTLE_OUT2, currentOut2);
    
    last_time = millis();
}

void loop() {
    unsigned long current_time = millis();
    
    // Only process if sample time has elapsed
    if (current_time - last_time >= SAMPLE_TIME_MS) {
        float dt = SAMPLE_TIME_MS / 1000.0;  // Convert to seconds
        
        // Read desired speed, current speed, and brake command from Python
        if (Serial.available() >= 3 * sizeof(float)) {
            Serial.readBytes((char*)&desired_speed, sizeof(float));
            Serial.readBytes((char*)&current_speed, sizeof(float));
            float brake_command;
            Serial.readBytes((char*)&brake_command, sizeof(float));
            
            float throttle = 0.0;
            
            // Implement mutual exclusion between throttle and brake
            if (brake_command > 0) {
                // If brake is commanded, ensure throttle is zero
                throttle = 0.0;
                setThrottle(0.0);
                setBrake(brake_command);
            } else {
                // If no brake, calculate and apply throttle
                throttle = calculateThrottle(dt);
                setThrottle(throttle);
                setBrake(0.0);
            }
            
            // Send control outputs back to Python
            Serial.write((char*)&throttle, sizeof(float));
            Serial.write((char*)&brake_command, sizeof(float));
        }
        
        last_time = current_time;
    }
}