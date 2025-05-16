#include <Arduino.h>

// Torque Output Pins
#define STEER_OUT1 PA4  // Steering DAC output 1
#define STEER_OUT2 PA5  // Steering DAC output 2
#define POTENTIOMETER_PIN A2  // Steering angle feedback

// Constants
const int SAMPLING_PERIOD = 20;       // 20ms = 50Hz sampling rate
const int ADC_MAX = 1023;             // 10-bit ADC max value
const int ADC_CENTER = ADC_MAX / 2;   // Center position (511 for 10-bit)
const float DAC_CENTER = 2.5;         // Center voltage (2.5V)
const float DAC_MAX_VOLTAGE = 3.3;    // Maximum DAC voltage
const float MAX_VOLTAGE_DELTA = 0.8;  // Maximum voltage change from center

// PID Controller Parameters
struct PIDController {
    float Kp = 2.0;    // Proportional gain
    float Ki = 1.5;    // Integral gain
    float Kd = 0.0;    // Derivative gain
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
const float DEAD_ZONE = 0.05;     // Output dead zone in volts

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    // Configure analog pin
    pinMode(POTENTIOMETER_PIN, INPUT);
    
    // Set analog and DAC resolution
    analogReadResolution(12);   // 12-bit ADC resolution
    analogWriteResolution(12);  // 12-bit DAC resolution
    
    // Initialize steering to center position
    setTorqueOutput(0.0);
    
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
    // Adjusting for potentially different ADC resolution
    float centerValue = (1 << (analogReadResolution(0) - 1)) - 1;
    return ((float)(rawValue - centerValue) / centerValue) * 200.0;
}

void setTorqueOutput(float pidOutput) {
    // Map PID output (-255 to 255) to voltage differential (-MAX_VOLTAGE_DELTA to +MAX_VOLTAGE_DELTA)
    float voltageDelta = (pidOutput / 255.0) * MAX_VOLTAGE_DELTA;
    
    // Apply dead zone
    if (abs(voltageDelta) < DEAD_ZONE) {
        voltageDelta = 0.0;
    }
    
    // Calculate output voltages
    float voltage1 = DAC_CENTER + voltageDelta;
    float voltage2 = DAC_CENTER - voltageDelta;
    
    // Constrain to valid voltage range
    voltage1 = constrain(voltage1, DAC_CENTER - MAX_VOLTAGE_DELTA, DAC_CENTER + MAX_VOLTAGE_DELTA);
    voltage2 = constrain(voltage2, DAC_CENTER - MAX_VOLTAGE_DELTA, DAC_CENTER + MAX_VOLTAGE_DELTA);
    
    // Convert to DAC values (12-bit: 0-4095)
    int dacValue1 = (voltage1 / DAC_MAX_VOLTAGE) * 4095;
    int dacValue2 = (voltage2 / DAC_MAX_VOLTAGE) * 4095;
    
    // Output to DACs
    analogWrite(STEER_OUT1, dacValue1);
    analogWrite(STEER_OUT2, dacValue2);
    
    Serial.print(" | V1: ");
    Serial.print(voltage1, 2);
    Serial.print("V, V2: ");
    Serial.print(voltage2, 2);
    Serial.print("V");
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
        
        // Apply control output to torque output
        setTorqueOutput(pidOutput);
        
        // Print debug information
        Serial.print("Target: ");
        Serial.print(targetAngle, 1);
        Serial.print(" | Current: ");
        Serial.print(currentAngle, 1);
        Serial.print(" | Error: ");
        Serial.print(error, 1);
        Serial.print(" | PID: ");
        Serial.print(pidOutput, 1);
        Serial.println();
    }
    
    // Check for new target angle from serial
    processSerial();
}