#include <Arduino.h>
#include <Servo.h>

// Pin Configuration
const int THROTTLE_OUT1 = PB10;  // DAC1 for throttle
const int THROTTLE_OUT2 = PB4;   // DAC2 for throttle
const int BRAKE_SERVO = PA9;     // Servo for brake control
const int STEER_DAC1 = PA4;      // DAC1 for steering (PA4)
const int STEER_DAC2 = PA5;      // DAC2 for steering (PA5)

// Throttle Variables
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 2V
const float maxOut2 = 1.0;   // 1V
const float dacMaxVoltage = 3.3;
const int throttleDacResolution = 1023; // 10-bit resolution
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// Brake Variables
Servo brakeServo;
const int BRAKE_RELEASED = 0;    // Angle for fully released brake
const int BRAKE_PRESSED = 180;   // Angle for fully pressed brake
int currentBrakeAngle = BRAKE_RELEASED;

// Steering Variables
const float STEER_IDLE_VOLTAGE = 2.5;  // Idle state at 2.5V
const float STEER_MIN_VOLTAGE = 0.0;   // Minimum voltage
const float STEER_MAX_VOLTAGE = 3.3;   // Maximum voltage
const int steerDacResolution = 1023;   // 10-bit resolution
float steerVoltage1 = STEER_IDLE_VOLTAGE;
float steerVoltage2 = STEER_IDLE_VOLTAGE;
float delta_steer = 0.0;
float desired_delta_steer = 0.0;
float steer_error = 0.0;
float last_steer_error = 0.0;
float steer_error_sum = 0.0;

// Smoothing for steering DAC outputs
static float smoothed_steer1 = STEER_IDLE_VOLTAGE;
static float smoothed_steer2 = STEER_IDLE_VOLTAGE;
const float alpha = 0.1; // Smoothing factor

// Controller gains (adjusted for stability)
struct ControllerGains {
    float Kp;
    float Ki;
    float Kd;
};

ControllerGains throttleGains = {1.3918, 3.9967, 0.0};
ControllerGains brakeGains = {5.0, 1.0, 0.0};
ControllerGains steerGains = {2.0, 0.05, 0.0}; // Reduced Kp, Ki for stability

// Speed Control Variables
float desired_speed = 0.0;
float current_speed = 0.0;
float speed_error = 0.0;
float last_speed_error = 0.0;
float speed_error_sum = 0.0;

// Dead zones
const float SPEED_DEADZONE = 0.5;  // ±0.5 kph
const float STEER_DEADZONE = 0.05; // ±0.05 meters

// Serial timeout
const unsigned long SERIAL_TIMEOUT_MS = 100;
unsigned long last_serial_time = 0;

// Function to set voltage
void setVoltage(int dacPin, float voltage, int resolution) {
    int dacValue = (voltage / dacMaxVoltage) * resolution;
    dacValue = constrain(dacValue, 0, resolution);
    analogWrite(dacPin, dacValue);
}

// Set throttle position
void setThrottle(float throttle_percent) {
    throttle_percent = constrain(throttle_percent, 0, 100); // Ensure valid range
    float normalized = throttle_percent / 100.0;
    currentOut2 = idleOut2 + normalized * (maxOut2 - idleOut2);
    currentOut1 = currentOut2 * 2;
    
    currentOut1 = constrain(currentOut1, idleOut1, maxOut1);
    currentOut2 = constrain(currentOut2, idleOut2, maxOut2);
    
    setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
    setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);
}

// Set brake position
void setBrake(float brake_percent) {
    brake_percent = constrain(brake_percent, 0, 100); // Ensure valid range
    int angle = map(brake_percent, 0, 100, BRAKE_RELEASED, BRAKE_PRESSED);
    brakeServo.write(angle);
    currentBrakeAngle = angle;
}

// Set steering voltages with strict idle state enforcement
void setSteering(float steer_control) {
    if (abs(steer_control) < 1e-6) { // If control is zero (within dead zone or no input)
        steerVoltage1 = STEER_IDLE_VOLTAGE;
        steerVoltage2 = STEER_IDLE_VOLTAGE;
        smoothed_steer1 = STEER_IDLE_VOLTAGE; // Reset smoothing to idle
        smoothed_steer2 = STEER_IDLE_VOLTAGE;
    } else {
        // Allow larger steering adjustments, but ensure voltages stay within bounds
        float voltage_adjustment = steer_control * (STEER_MAX_VOLTAGE - STEER_IDLE_VOLTAGE);
        
        steerVoltage1 = STEER_IDLE_VOLTAGE + voltage_adjustment;
        steerVoltage2 = STEER_IDLE_VOLTAGE - voltage_adjustment;
        
        // Constrain voltages to hardware limits
        steerVoltage1 = constrain(steerVoltage1, STEER_MIN_VOLTAGE, STEER_MAX_VOLTAGE);
        steerVoltage2 = constrain(steerVoltage2, STEER_MIN_VOLTAGE, STEER_MAX_VOLTAGE);
        
        // Apply smoothing
        smoothed_steer1 = (1 - alpha) * smoothed_steer1 + alpha * steerVoltage1;
        smoothed_steer2 = (1 - alpha) * smoothed_steer2 + alpha * steerVoltage2;
    }
    
    setVoltage(STEER_DAC1, smoothed_steer1, steerDacResolution);
    setVoltage(STEER_DAC2, smoothed_steer2, steerDacResolution);
}

// Reset to idle state
void resetToIdle() {
    setThrottle(0);
    setBrake(0);
    setSteering(0);
}

// Structure for control outputs
struct ControlOutputs {
    float throttle_percent;
    float brake_percent;
    float steer_control;
};

// Calculate MIMO control outputs
ControlOutputs calculateControl(float dt) {
    ControlOutputs outputs = {0, 0, 0};
    
    // Speed control
    speed_error = desired_speed - current_speed;
    
    if (abs(speed_error) < SPEED_DEADZONE) {
        speed_error_sum = 0;
        outputs.throttle_percent = 0;
        outputs.brake_percent = 0;
    } else {
        float speed_error_rate = (speed_error - last_speed_error) / dt;
        speed_error_sum += speed_error * dt;
        
        if (speed_error > 0) {
            float control = throttleGains.Kp * speed_error + 
                           throttleGains.Ki * speed_error_sum + 
                           throttleGains.Kd * speed_error_rate;
            outputs.throttle_percent = constrain(control, 0, 100);
            outputs.brake_percent = 0;
        } else {
            float control = -(brakeGains.Kp * speed_error + 
                             brakeGains.Ki * speed_error_sum + 
                             brakeGains.Kd * speed_error_rate);
            outputs.brake_percent = constrain(control, 0, 100);
            outputs.throttle_percent = 0;
        }
    }
    
    last_speed_error = speed_error;
    
    // Steering control
    steer_error = desired_delta_steer - delta_steer;
    
    if (abs(steer_error) < STEER_DEADZONE) {
        steer_error_sum = 0;
        outputs.steer_control = 0;
    } else {
        float steer_error_rate = (steer_error - last_steer_error) / dt;
        steer_error_sum += steer_error * dt;
        
        float control = steerGains.Kp * steer_error + 
                       steerGains.Ki * steer_error_sum + 
                       steerGains.Kd * steer_error_rate;
        outputs.steer_control = constrain(control, -1.0, 1.0);
        outputs.steer_control = control;
    }
    
    last_steer_error = steer_error;
    
    return outputs;
}

void setup() {
    Serial.begin(9600);
    while (!Serial) delay(10);
    
    // Initialize brake servo
    brakeServo.attach(BRAKE_SERVO);
    brakeServo.write(BRAKE_RELEASED);
    
    // Initialize throttle outputs
    setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
    setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);
    
    // Initialize steering outputs
    setVoltage(STEER_DAC1, STEER_IDLE_VOLTAGE, steerDacResolution);
    setVoltage(STEER_DAC2, STEER_IDLE_VOLTAGE, steerDacResolution);
    
    // Set DAC resolution
    analogWriteResolution(10);
    
    last_serial_time = millis();
}

void loop() {
    unsigned long start_time = micros();
    float dt = 0.01; // 10ms in seconds
    
    // Check for Serial timeout
    if (millis() - last_serial_time > SERIAL_TIMEOUT_MS) {
        resetToIdle();
        Serial.println("Serial timeout, resetting to idle");
    }
    
    // Read Serial data with synchronization
    if (Serial.available() >= 1 + 4 * sizeof(float)) {
        // Flush serial buffer to clear any stale data
        Serial.flush();
        
        char syncByte = Serial.read();
        if (syncByte == 'S') {
            float temp[4];
            Serial.readBytes((char*)temp, 4 * sizeof(float));
            
            // Validate inputs
            desired_speed = temp[0];
            current_speed = temp[1];
            delta_steer = temp[2];  // No constraint
            desired_delta_steer = temp[3];  // No constraint
            
            // Constrain speeds to reasonable ranges
            desired_speed = constrain(desired_speed, 0, 100); // Example: 0-100 kph
            current_speed = constrain(current_speed, 0, 100);
            // Removed constraints on delta_steer and desired_delta_steer
            delta_steer = constrain(delta_steer, -1.0, 1.0);
            desired_delta_steer = constrain(desired_delta_steer, -1.0, 1.0);
            
            // Calculate and apply control
            ControlOutputs outputs = calculateControl(dt);
            setThrottle(outputs.throttle_percent);
            setBrake(outputs.brake_percent);
            setSteering(outputs.steer_control);
            
            // Log data in CSV format with additional debug info
            unsigned long time_ms = millis();
            Serial.print(time_ms);
            Serial.print(",");
            Serial.print(currentOut1, 2);
            Serial.print(",");
            Serial.print(currentOut2, 2);
            Serial.print(",");
            Serial.print(currentBrakeAngle);
            Serial.print(",");
            Serial.print(smoothed_steer1, 2);
            Serial.print(",");
            Serial.print(smoothed_steer2, 2);
            Serial.print(",");
            Serial.print(delta_steer, 2);
            Serial.print(",");
            Serial.print(desired_delta_steer, 2);
            Serial.print(",");
            Serial.print(outputs.throttle_percent, 2);
            Serial.print(",");
            Serial.print(outputs.brake_percent, 2);
            Serial.print(",");
            Serial.print(steer_error, 2); // Added for debugging
            Serial.print(",");
            Serial.print(outputs.steer_control, 2); // Added for debugging
            Serial.println();
            
            // Validate throttle and brake before sending
            float throttle_to_send = constrain(outputs.throttle_percent, 0, 100);
            float brake_to_send = constrain(outputs.brake_percent, 0, 100);
            
            // Send control outputs to Python
            Serial.write((char*)&throttle_to_send, sizeof(float));
            Serial.write((char*)&brake_to_send, sizeof(float));
            
            last_serial_time = millis();
        } else {
            // If sync byte is incorrect, flush the buffer to resynchronize
            Serial.flush();
            Serial.println("Invalid sync byte, resynchronizing");
        }
    }
    
    // Check loop timing
    unsigned long elapsed = micros() - start_time;
    if (elapsed > 10000) { // 10ms in microseconds
        Serial.print("Warning: Loop time exceeded: ");
        Serial.print(elapsed);
        Serial.println(" us");
    }
    
    delay(10); // Enforce 10ms loop
}