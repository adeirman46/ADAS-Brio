#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Pin Configuration
const int THROTTLE_OUT1 = PB10;  // DAC1
const int THROTTLE_OUT2 = PB4;   // DAC2
const int BRAKE_SERVO = PA9;     // Servo for brake control
const int STEER_OUT1 = PA4;      // Steering DAC output 1
const int STEER_OUT2 = PA5;      // Steering DAC output 2
const int pedalPin = PA7;        // Analog input for throttle pedal
const int STEER_POT = PB0;       // Potentiometer for steering
const int STEER_ANALOG1 = A0;    // ADC input for steering DAC1
const int STEER_ANALOG2 = A1;    // ADC input for steering DAC2

// VL53L0X for brake distance measurement
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
const float BRAKE_RELEASED_MM = 114.0; // 11.4cm when pedal not pressed (0%)
const float BRAKE_FULL_MM = 52.0;      // 5.2cm when pedal fully pressed (100%)
const float BRAKE_RANGE_MM = BRAKE_RELEASED_MM - BRAKE_FULL_MM; // Travel range

// Mode control
bool manualMode = true;          // Default: Manual Mode

// Throttle Variables
const float idleOut1 = 1.0;      // 1V
const float idleOut2 = 0.5;      // 0.5V
const float maxOut1 = 2.0;       // 2V
const float maxOut2 = 1.0;       // 1V
const float dacMaxVoltage = 3.3;
const int throttleDacResolution = 1023; // 10-bit resolution for throttle
const int steerDacResolution = 1023;    // 10-bit resolution for steering
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// IIR Filter parameters for throttle
const float throttleWc = 0.6302 * 2 * PI; // Cutoff frequency in rad/s
const float throttleTs = 0.1;    // Sampling period in seconds
float throttle_b0, throttle_b1, throttle_b2, throttle_a1, throttle_a2;
float throttle_prev_input_1 = 0.0;
float throttle_prev_input_2 = 0.0;
float throttle_filtered_value = 0.0;
float throttle_prev_filtered_1 = 0.0;
float throttle_prev_filtered_2 = 0.0;

// Brake Servo Variables
Servo brakeServo;
int brakeValue = 0;
const int brakeStep = 18;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;

// IIR Filter parameters for brake distance
const float brakeWc = 0.3 * 2 * PI; // Cutoff frequency in rad/s
const float brakeTs = 0.1;          // Sampling period in seconds
float brake_b0, brake_b1, brake_b2, brake_a1, brake_a2;
float brake_prev_input_1 = 0.0;
float brake_prev_input_2 = 0.0;
float brake_filtered_distance = 0.0;
float brake_prev_filtered_1 = 0.0;
float brake_prev_filtered_2 = 0.0;

// Steering Variables
float steerVoltage1 = 2.5;       // Center position (2.5V)
float steerVoltage2 = 2.5;       // Center position (2.5V)
const int STEER_IDLE_STATE = 775; // 2.5V on 10-bit scale (2.5/3.3*1023)
const int STEER_ADC_MAX = 1023;   // 10-bit ADC max value
const int STEER_ADC_CENTER = 512; // Center position for potentiometer (10-bit)
const float degrees_per_adc_ccw = 90.0 / 40.0; // 2.25 deg/ADC
const float degrees_per_adc_cw = 90.0 / 40.0;  // 2.25 deg/ADC
// IIR Filter parameters for steering
const float steerWc = 2.8629;    // Cutoff frequency in rad/s
const float steerTs = 0.1;       // Sampling period in seconds
float steer_b0, steer_b1, steer_b2, steer_a1, steer_a2;
float steer_prev_input_1 = 0.0;
float steer_prev_input_2 = 0.0;
float steer_filtered_value = 0.0;
float steer_prev_filtered_1 = 0.0;
float steer_prev_filtered_2 = 0.0;

// Variabel untuk smoothing di manual mode (steering)
static float smoothed_val1 = STEER_IDLE_STATE;
static float smoothed_val2 = STEER_IDLE_STATE;
const float alpha = 0.1; // Faktor smoothing (0–1, kecil = lebih halus)

// Function to set voltage (supports different resolutions)
void setVoltage(int dacPin, float voltage, int resolution) {
  int dacValue = (voltage / dacMaxVoltage) * resolution;
  dacValue = constrain(dacValue, 0, resolution);
  analogWrite(dacPin, dacValue);
}

// Function to calculate brake pedal position as a percentage
float calculateBrakePercentage(float distance_mm) {
  if (distance_mm > BRAKE_RELEASED_MM) {
    distance_mm = BRAKE_RELEASED_MM; // Cap at maximum distance (0% pressed)
  } else if (distance_mm < BRAKE_FULL_MM) {
    distance_mm = BRAKE_FULL_MM;     // Cap at minimum distance (100% pressed)
  }
  float percentage = ((BRAKE_RELEASED_MM - distance_mm) / BRAKE_RANGE_MM) * 100.0;
  return percentage;
}

// Initialize all components
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  // Initialize Brake Servo
  brakeServo.attach(BRAKE_SERVO);
  brakeServo.write(MIN_ANGLE);

  // Initialize VL53L0X
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X sensor"));
    while (1); // Halt if sensor initialization fails
  }
  lox.startRangeContinuous();

  // Initialize pins
  pinMode(pedalPin, INPUT);
  pinMode(STEER_POT, INPUT);
  pinMode(STEER_ANALOG1, INPUT);
  pinMode(STEER_ANALOG2, INPUT);
  analogReadResolution(10); // 10-bit for all ADC inputs
  analogWriteResolution(10); // 10-bit for all DAC outputs

  // Set initial idle state for throttle
  currentOut1 = idleOut1;
  currentOut2 = idleOut2;
  setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
  setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);

  // Set initial center position for steering
  setVoltage(STEER_OUT1, 2.5, steerDacResolution);
  setVoltage(STEER_OUT2, 2.5, steerDacResolution);

  // Calculate IIR filter coefficients for throttle
  float alpha_iir = (throttleWc * throttleTs) / 2.0;
  float denom = 1.0 + sqrt(2.0) * alpha_iir + alpha_iir * alpha_iir;
  throttle_a1 = -2.0 * (1.0 - alpha_iir * alpha_iir) / denom;
  throttle_a2 = (1.0 - sqrt(2.0) * alpha_iir + alpha_iir * alpha_iir) / denom;
  throttle_b0 = (alpha_iir * alpha_iir) / denom;
  throttle_b1 = 2.0 * throttle_b0;
  throttle_b2 = throttle_b0;

  // Initialize throttle filter with idle assumption
  float throttle_initial_value = 140; // Assume idle position
  throttle_prev_input_1 = throttle_initial_value;
  throttle_prev_input_2 = throttle_initial_value;
  throttle_filtered_value = throttle_initial_value;
  throttle_prev_filtered_1 = throttle_filtered_value;
  throttle_prev_filtered_2 = throttle_filtered_value;

  // Calculate IIR filter coefficients for brake
  alpha_iir = (brakeWc * brakeTs) / 2.0;
  denom = 1.0 + sqrt(2.0) * alpha_iir + alpha_iir * alpha_iir;
  brake_a1 = -2.0 * (1.0 - alpha_iir * alpha_iir) / denom;
  brake_a2 = (1.0 - sqrt(2.0) * alpha_iir + alpha_iir * alpha_iir) / denom;
  brake_b0 = (alpha_iir * alpha_iir) / denom;
  brake_b1 = 2.0 * brake_b0;
  brake_b2 = brake_b0;

  // Initialize brake filter
  float brake_initial_value = BRAKE_RELEASED_MM;
  if (lox.isRangeComplete()) {
    brake_initial_value = lox.readRange();
  }
  brake_prev_input_1 = brake_initial_value;
  brake_prev_input_2 = brake_initial_value;
  brake_filtered_distance = brake_initial_value;
  brake_prev_filtered_1 = brake_filtered_distance;
  brake_prev_filtered_2 = brake_filtered_distance;

  // Calculate IIR filter coefficients for steering
  alpha_iir = (steerWc * steerTs) / 2.0;
  denom = 1.0 + sqrt(2.0) * alpha_iir + alpha_iir * alpha_iir;
  steer_a1 = -2.0 * (1.0 - alpha_iir * alpha_iir) / denom;
  steer_a2 = (1.0 - sqrt(2.0) * alpha_iir + alpha_iir * alpha_iir) / denom;
  steer_b0 = (alpha_iir * alpha_iir) / denom;
  steer_b1 = 2.0 * steer_b0;
  steer_b2 = steer_b0;

  // Initialize steering filter
  float steer_initial_value = analogRead(STEER_POT);
  steer_prev_input_1 = steer_initial_value;
  steer_prev_input_2 = steer_initial_value;
  steer_filtered_value = steer_initial_value;
  steer_prev_filtered_1 = steer_filtered_value;
  steer_prev_filtered_2 = steer_filtered_value;

  Serial.println("System initialized. Mode: Manual (default)");
  Serial.println("Commands: 'm' toggle mode, 'w'/'s' throttle, 'b'/'n' brake, 'a'/'d' steering");
  Serial.print("Initial throttle: Out1=");
  Serial.print(currentOut1, 2);
  Serial.print("V, Out2=");
  Serial.print(currentOut2, 2);
  Serial.println("V");
}

void loop() {
  // Handle serial input
  if (Serial.available() > 0) {
    char command = Serial.read();

    // Mode toggle
    if (command == 'm' || command == 'M') {
      manualMode = !manualMode;
      // Reset to idle/center states
      currentOut1 = idleOut1;
      currentOut2 = idleOut2;
      setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
      setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);
      steerVoltage1 = 2.5;
      steerVoltage2 = 2.5;
      setVoltage(STEER_OUT1, steerVoltage1, steerDacResolution);
      setVoltage(STEER_OUT2, steerVoltage2, steerDacResolution);
      brakeValue = MIN_ANGLE;
      brakeServo.write(brakeValue);
      Serial.print("Mode: ");
      Serial.println(manualMode ? "Manual" : "ADAS");
    }

    // Throttle Control (ADAS mode)
    if (!manualMode && (command == 'w' || command == 'W')) {
      if (currentOut2 < maxOut2) {
        currentOut2 += 0.1;
        currentOut1 = currentOut2 * 2;
        if (currentOut1 > maxOut1) {
          currentOut1 = maxOut1;
        }
        setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
        setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);
        Serial.print("Increased - Out1: ");
        Serial.print(currentOut1, 2);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2, 2);
        Serial.println("V");
      } else {
        Serial.println("Maximum throttle voltage reached");
      }
    } else if (!manualMode && (command == 's' || command == 'S')) {
      if (currentOut2 > idleOut2) {
        currentOut2 -= 0.1;
        currentOut1 = currentOut2 * 2;
        if (currentOut1 < idleOut1) {
          currentOut1 = idleOut1;
        }
        setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
        setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);
        Serial.print("Decreased - Out1: ");
        Serial.print(currentOut1, 2);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2, 2);
        Serial.println("V");
      } else {
        Serial.println("Minimum throttle voltage reached");
      }
    }

    // Brake Control (ADAS mode)
    if (!manualMode && command == 'b') {
      if (brakeValue <= (MAX_ANGLE - brakeStep)) {
        brakeValue += brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print((brakeValue * 100) / ANGLE_RANGE);
        Serial.println("%");
      }
    } else if (!manualMode && command == 'n') {
      if (brakeValue >= (MIN_ANGLE + brakeStep)) {
        brakeValue -= brakeStep;
        brakeServo.write(brakeValue);
        Serial.print("Brake at: ");
        Serial.print((brakeValue * 100) / ANGLE_RANGE);
        Serial.println("%");
      }
    }

    // Steering Control (ADAS mode)
    if (!manualMode && (command == 'a' || command == 'A')) {
      if (steerVoltage1 < 3.3 && steerVoltage2 > 0.0) {
        steerVoltage1 += 0.1;
        steerVoltage2 -= 0.1;
      }
      setVoltage(STEER_OUT1, steerVoltage1, steerDacResolution);
      setVoltage(STEER_OUT2, steerVoltage2, steerDacResolution);
      Serial.print("DAC Output - PA4: ");
      Serial.print(steerVoltage1, 2);
      Serial.print("V, PA5: ");
      Serial.print(steerVoltage2, 2);
      Serial.println("V");
    } else if (!manualMode && (command == 'd' || command == 'D')) {
      if (steerVoltage1 > 0.0 && steerVoltage2 < 3.3) {
        steerVoltage1 -= 0.1;
        steerVoltage2 += 0.1;
      }
      setVoltage(STEER_OUT1, steerVoltage1, steerDacResolution);
      setVoltage(STEER_OUT2, steerVoltage2, steerDacResolution);
      Serial.print("DAC Output - PA4: ");
      Serial.print(steerVoltage1, 2);
      Serial.print("V, PA5: ");
      Serial.print(steerVoltage2, 2);
      Serial.println("V");
    }
  }

  // Throttle: Read and process pedal value
  float throttleRawValue = analogRead(pedalPin);
  // Apply filtering for logging and potential ADAS use
  throttle_filtered_value = throttle_b0 * throttleRawValue + throttle_b1 * throttle_prev_input_1 + 
                           throttle_b2 * throttle_prev_input_2 - throttle_a1 * throttle_prev_filtered_1 - 
                           throttle_a2 * throttle_prev_filtered_2;
  // Update previous values
  throttle_prev_input_2 = throttle_prev_input_1;
  throttle_prev_input_1 = throttleRawValue;
  throttle_prev_filtered_2 = throttle_prev_filtered_1;
  throttle_prev_filtered_1 = throttle_filtered_value;

  // Calculate pedal percentage based on filtered value
  float pedalPercentage = map(throttle_filtered_value, 140, 500, 0, 100);
  pedalPercentage = constrain(pedalPercentage, 0, 100);

  // In manual mode, use rawValue for immediate control
  if (manualMode) {
    float pedalVoltage = map(throttleRawValue, 140, 500, idleOut2 * 1000, maxOut2 * 1000) / 1000.0;
    pedalVoltage = constrain(pedalVoltage, idleOut2, maxOut2);
    currentOut2 = pedalVoltage;
    currentOut1 = currentOut2 * 2;
    if (currentOut1 < idleOut1) currentOut1 = idleOut1;
    if (currentOut1 > maxOut1) currentOut1 = maxOut1;
    setVoltage(THROTTLE_OUT1, currentOut1, throttleDacResolution);
    setVoltage(THROTTLE_OUT2, currentOut2, throttleDacResolution);
  } else {
    // Ensure idle state in ADAS mode unless commanded
    if (currentOut1 == idleOut1 && currentOut2 == idleOut2) {
      setVoltage(THROTTLE_OUT1, idleOut1, throttleDacResolution);
      setVoltage(THROTTLE_OUT2, idleOut2, throttleDacResolution);
    }
  }

  // Brake: Read and filter distance
  float brakeRawDistance = BRAKE_RELEASED_MM;
  float brakePercentage = 0.0;
  if (lox.isRangeComplete()) {
    brakeRawDistance = lox.readRange();
    brake_filtered_distance = brake_b0 * brakeRawDistance + brake_b1 * brake_prev_input_1 + 
                             brake_b2 * brake_prev_input_2 - brake_a1 * brake_prev_filtered_1 - 
                             brake_a2 * brake_prev_filtered_2;
    // Update previous values
    brake_prev_input_2 = brake_prev_input_1;
    brake_prev_input_1 = brakeRawDistance;
    brake_prev_filtered_2 = brake_prev_filtered_1;
    brake_prev_filtered_1 = brake_filtered_distance;
    brakePercentage = calculateBrakePercentage(brake_filtered_distance);
  }

  // Steering: Read and filter potentiometer
  float steerRawValue = analogRead(STEER_POT);
  steer_filtered_value = steer_b0 * steerRawValue + steer_b1 * steer_prev_input_1 + 
                        steer_b2 * steer_prev_input_2 - steer_a1 * steer_prev_filtered_1 - 
                        steer_a2 * steer_prev_filtered_2;
  // Update previous values
  steer_prev_input_2 = steer_prev_input_1;
  steer_prev_input_1 = steerRawValue;
  steer_prev_filtered_2 = steer_prev_filtered_1;
  steer_prev_filtered_1 = steer_filtered_value;
  
  // Calculate steering angle
  float deviation = STEER_ADC_CENTER - steer_filtered_value;
  float steeringAngle;
  if (deviation > 0) {
    steeringAngle = deviation * degrees_per_adc_ccw;
  } else {
    steeringAngle = deviation * degrees_per_adc_cw;
  }
  
  // Clamp steering angle to ±720 degrees
  if (steeringAngle > 720.0) steeringAngle = 720.0;
  if (steeringAngle < -720.0) steeringAngle = -720.0;
  
  // Handle steering DAC in manual mode
  if (manualMode) {
    int val1 = analogRead(STEER_ANALOG1);
    int val2 = analogRead(STEER_ANALOG2);
    
    // Apply low-pass filter
    smoothed_val1 = (1 - alpha) * smoothed_val1 + alpha * val1;
    smoothed_val2 = (1 - alpha) * smoothed_val2 + alpha * val2;
    
    // Write to DAC
    analogWrite(STEER_OUT1, val1 > 0 ? (int)smoothed_val1 : STEER_IDLE_STATE);
    analogWrite(STEER_OUT2, val2 > 0 ? (int)smoothed_val2 : STEER_IDLE_STATE);
    steerVoltage1 = (smoothed_val1 / 1023.0) * 3.3;
    steerVoltage2 = (smoothed_val2 / 1023.0) * 3.3;
  }

  // Log data in CSV format: time_ms,throttle_out1,throttle_out2,mode,throttle_raw_value,throttle_filtered_value,pedal_percentage,brake_raw_distance,brake_filtered_distance,brake_percentage,steer_voltage1,steer_voltage2,steering_angle
  unsigned long time_ms = millis();
  Serial.print(time_ms);
  Serial.print(",");
  Serial.print(currentOut1, 2);
  Serial.print(",");
  Serial.print(currentOut2, 2);
  Serial.print(",");
  Serial.print(manualMode ? "Manual" : "ADAS");
  Serial.print(",");
  Serial.print(throttleRawValue);
  Serial.print(",");
  Serial.print(throttle_filtered_value);
  Serial.print(",");
  Serial.print(pedalPercentage);
  Serial.print(",");
  Serial.print(brakeRawDistance);
  Serial.print(",");
  Serial.print(brake_filtered_distance);
  Serial.print(",");
  Serial.print(brakePercentage);
  Serial.print(",");
  Serial.print(steerVoltage1, 2);
  Serial.print(",");
  Serial.print(steerVoltage2, 2);
  Serial.print(",");
  Serial.print(steeringAngle, 1);
  Serial.println();
  
  delay(10); // Controlled logging rate
}

