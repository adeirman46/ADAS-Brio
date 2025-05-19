#include <Arduino.h>

// Pin definitions
const int pedalPin = PA7;    // Analog input for pedal
const int out1Pin = PB10;    // DAC1
const int out2Pin = PB4;     // DAC2

// Mode control
bool manualMode = true;      // Default: Manual Mode

// Target voltages for idle and max states
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 2V
const float maxOut2 = 1.0;   // 1V

// Voltage range for DAC on STM32F446RE
const float dacMaxVoltage = 3.3;
const int dacResolution = 1023; // 10-bit resolution

// Current output voltages
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// IIR Filter parameters for pedal
const float Wc = 0.6302*2*PI;     // Cutoff frequency in rad/s
const float Ts = 0.1;        // Sampling period in seconds

// IIR filter coefficients
float b0, b1, b2, a1, a2;

// Buffers for previous inputs and outputs
float prev_input_1 = 0.0;
float prev_input_2 = 0.0;
float filtered_value = 0.0;
float prev_filtered_1 = 0.0;
float prev_filtered_2 = 0.0;

// Helper function to set the output voltage on a DAC pin
void setVoltage(int dacPin, float voltage) {
  int dacValue = (voltage / dacMaxVoltage) * dacResolution;
  dacValue = constrain(dacValue, 0, dacResolution);
  analogWrite(dacPin, dacValue);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
  
  // Initialize pins
  pinMode(pedalPin, INPUT);
  analogReadResolution(10);  // Set analog read resolution to 10 bits
  analogWriteResolution(10); // Set analog write resolution to 10 bits
  
  // Set initial idle state
  setVoltage(out1Pin, currentOut1);
  setVoltage(out2Pin, currentOut2);
  
  // Calculate IIR filter coefficients
  float alpha = (Wc * Ts) / 2.0;
  float denom = 1.0 + sqrt(2.0) * alpha + alpha * alpha;
  a1 = -2.0 * (1.0 - alpha * alpha) / denom;
  a2 = (1.0 - sqrt(2.0) * alpha + alpha * alpha) / denom;
  b0 = (alpha * alpha) / denom;
  b1 = 2.0 * b0;
  b2 = b0;
  
  // Initialize filter with first reading
  float initial_value = analogRead(pedalPin);
  prev_input_1 = initial_value;
  prev_input_2 = initial_value;
  filtered_value = initial_value;
  prev_filtered_1 = filtered_value;
  prev_filtered_2 = filtered_value;
  
  Serial.println("Throttle Control System Initialized");
  Serial.println("Mode: Manual (default)");
  Serial.println("Press 'm' to switch mode, 'w'/'s' for ADAS control");
}

void loop() {
  // Handle serial input for mode and ADAS control
  if (Serial.available()) {
    char input = Serial.read();
    
    if (input == 'm' || input == 'M') {
      manualMode = !manualMode;
      // Reset to idle voltages on mode switch
      currentOut1 = idleOut1;
      currentOut2 = idleOut2;
      setVoltage(out1Pin, currentOut1);
      setVoltage(out2Pin, currentOut2);
      Serial.print("Mode: ");
      Serial.println(manualMode ? "Manual" : "ADAS");
    }
    
    if (!manualMode) { // ADAS mode keyboard control
      if (input == 'w' || input == 'W') {
        if (currentOut2 < maxOut2) {
          currentOut2 += 0.1;
          currentOut1 = currentOut2 * 2;
          if (currentOut1 > maxOut1) {
            currentOut1 = maxOut1;
          }
          setVoltage(out1Pin, currentOut1);
          setVoltage(out2Pin, currentOut2);
          Serial.print("Increased - Out1: ");
          Serial.print(currentOut1);
          Serial.print("V, Out2: ");
          Serial.print(currentOut2);
          Serial.println("V");
        } else {
          Serial.println("Maximum voltage reached");
        }
      } else if (input == 's' || input == 'S') {
        if (currentOut2 > idleOut2) {
          currentOut2 -= 0.1;
          currentOut1 = currentOut2 * 2;
          if (currentOut1 < idleOut1) {
            currentOut1 = idleOut1;
          }
          setVoltage(out1Pin, currentOut1);
          setVoltage(out2Pin, currentOut2);
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
  }
  
  // Read pedal value
  float rawValue = analogRead(pedalPin);
  
  // Apply filtering for logging and potential ADAS use
  filtered_value = b0 * rawValue + b1 * prev_input_1 + b2 * prev_input_2 - a1 * prev_filtered_1 - a2 * prev_filtered_2;
  
  // Update previous values
  prev_input_2 = prev_input_1;
  prev_input_1 = rawValue;
  prev_filtered_2 = prev_filtered_1;
  prev_filtered_1 = filtered_value;
  
  // Calculate pedal percentage based on filtered value
  float pedalPercentage = map(filtered_value, 140, 500, 0, 100);
  pedalPercentage = constrain(pedalPercentage, 0, 100);
  
  // In manual mode, use rawValue for immediate control
  if (manualMode) {
    // Map rawValue (140 to 750) to voltage range (idleOut2 to maxOut2)
    float pedalVoltage = map(rawValue, 140, 500, idleOut2 * 1000, maxOut2 * 1000) / 1000.0;
    pedalVoltage = constrain(pedalVoltage, idleOut2, maxOut2);
    currentOut2 = pedalVoltage;
    currentOut1 = currentOut2 * 2;
    if (currentOut1 < idleOut1) currentOut1 = idleOut1;
    if (currentOut1 > maxOut1) currentOut1 = maxOut1;
    setVoltage(out1Pin, currentOut1);
    setVoltage(out2Pin, currentOut2);
  }
  
  // Log data in CSV format: time_ms,out1,out2,mode,raw_value,filtered_value,pedal_percentage
  unsigned long time_ms = millis();
  Serial.print(time_ms);
  Serial.print(",");
  Serial.print(currentOut1, 2);
  Serial.print(",");
  Serial.print(currentOut2, 2);
  Serial.print(",");
  Serial.print(manualMode ? "Manual" : "ADAS");
  Serial.print(",");
  Serial.print(rawValue);
  Serial.print(",");
  Serial.print(filtered_value);
  Serial.print(",");
  Serial.print(pedalPercentage);
  Serial.println();
  
  delay(100); // Controlled logging rate
}