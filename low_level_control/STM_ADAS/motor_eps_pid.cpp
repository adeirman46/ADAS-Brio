#include <Arduino.h>

// Pins for IBT-2 module
#define R_PWM D5  // RPWM pin - Forward PWM
#define L_PWM D6  // LPWM pin - Reverse PWM
#define R_EN  D3  // R_EN pin
#define L_EN  D4  // L_EN pin
#define POTENTIOMETER_PIN A2  // Steering angle feedback

// Constants
const int SAMPLING_PERIOD = 20;       // 20ms = 50Hz sampling rate
const int ADC_MAX = 1023;            // 10-bit ADC max value
const int ADC_CENTER = ADC_MAX / 2;  // Center position (511 for 10-bit)

// PID Controller Parameters
struct PIDController {
    float Kp = 2.0;    // Proportional gain
    float Ki = 1.5;    // Integral gain
    float Kd = 0.0;   // Derivative gain
    float integral = 0.0;
    float prev_error = 0.0;
    float output = 0.0;
};

// Global variables
PIDController pid;
unsigned long lastSampleTime = 0;
float targetAngle = 0.0;     // Desired steering angle (-200 to +200)
float currentAngle = 0.0;    // Current steering angle (-200 to +200)
const float MAX_INTEGRAL = 50.0;  // Anti-windup limit
const int DEAD_ZONE = 5;     // Motor dead zone compensation

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    // Configure pins
    pinMode(POTENTIOMETER_PIN, INPUT);
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
    
    Serial.println("EPS Position Controller Initialized");
    Serial.println("Send target angle (-200 to 200)");
}

float readSteeringAngle() {
    // Read potentiometer with averaging for noise reduction
    int rawValue = 0;
    for(int i = 0; i < 3; i++) {
        rawValue += analogRead(POTENTIOMETER_PIN);
        delayMicroseconds(100);
    }
    rawValue /= 3;
    
    // Convert to angle percentage (-200 to +200)
    return ((float)(rawValue - ADC_CENTER) / ADC_CENTER) * 200.0;
}

void setMotorOutput(float pidOutput) {
    // Convert PID output to motor PWM value (-255 to 255)
    int motorPWM = (int)constrain(pidOutput, -255, 255);
    
    // Apply dead zone compensation
    if (abs(motorPWM) < DEAD_ZONE) {
        motorPWM = 0;
    } else {
        if (motorPWM > 0) {
            motorPWM += DEAD_ZONE;
        } else {
            motorPWM -= DEAD_ZONE;
        }
    }
    
    // Set motor direction and speed
    if (motorPWM >= 0) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, motorPWM);
    } else {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, -motorPWM);
    }
}

float updatePID(float error, float dt) {
    // Calculate integral term with anti-windup
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -MAX_INTEGRAL, MAX_INTEGRAL);
    
    // Calculate derivative term
    float derivative = (error - pid.prev_error) / dt;
    
    // Calculate PID output
    pid.output = (pid.Kp * error) + 
                (pid.Ki * pid.integral) + 
                (pid.Kd * derivative);
    
    // Store error for next iteration
    pid.prev_error = error;
    
    return pid.output;
}

void processSerial() {
    if (Serial.available() > 0) {
        // Read the target angle from serial
        targetAngle = Serial.parseFloat();
        
        // Constrain target angle
        targetAngle = constrain(targetAngle, -200.0, 200.0);
        
        // Clear serial buffer
        while(Serial.available()) {
            Serial.read();
        }
        
        Serial.print("Target angle set to: ");
        Serial.println(targetAngle);
    }
}

void loop() {
    unsigned long currentTime = millis();
    
    // Update at fixed sampling rate
    if (currentTime - lastSampleTime >= SAMPLING_PERIOD) {
        float dt = (currentTime - lastSampleTime) / 1000.0;  // Convert to seconds
        lastSampleTime = currentTime;
        
        // Read current steering angle
        currentAngle = readSteeringAngle();
        
        // Calculate error
        float error = targetAngle - currentAngle;
        
        // Update PID controller
        float pidOutput = updatePID(error, dt);
        
        // Apply control output to motor
        setMotorOutput(pidOutput);
        
        // Print debug information
        Serial.print("Target: ");
        Serial.print(targetAngle, 1);
        Serial.print(" | Current: ");
        Serial.print(currentAngle, 1);
        Serial.print(" | Error: ");
        Serial.print(error, 1);
        Serial.print(" | Output: ");
        Serial.println(pidOutput, 1);
    }
    
    // Check for new target angle from serial
    processSerial();
}