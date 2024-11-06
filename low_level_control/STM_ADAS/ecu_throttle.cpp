// STM32F446RE has DAC on PA4 (DAC_OUT1) and PA5 (DAC_OUT2)

#include <Arduino.h>
const int out1Pin = D5; // DAC1
const int out2Pin = D6; // DAC2

// Target voltages for idle and max states
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 5V (assuming scaling)
const float maxOut2 = 1.0;   // 2.5V (assuming scaling)

// Voltage range for DAC on STM32F446RE is 0 to 3.3V
const float dacMaxVoltage = 3.3;
const int dacResolution = 4095; // 12-bit resolution (0-4095)


// Helper function to set the output voltage on a DAC pin
void setVoltage(int dacPin, float voltage) {
  // Scale the voltage to DAC value (12-bit resolution)
  int dacValue = (voltage / dacMaxVoltage) * dacResolution;
  
  // Ensure the DAC value stays within bounds
  dacValue = constrain(dacValue, 0, dacResolution);
  
  // Use analogWrite instead of dacWrite for STM32
  analogWrite(dacPin, dacValue);
}

// Direction control
bool rampingUp = true;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize DAC
  analogWriteResolution(12); // Set 12-bit resolution
  
  // Set initial idle state
  setVoltage(out1Pin, idleOut1);
  setVoltage(out2Pin, idleOut2);
  
  delay(15000);  // Hold idle state for 15 seconds
}

void loop() {
  if (rampingUp) {
    // Ramp up
    for (float out2Voltage = idleOut2; out2Voltage <= maxOut2; out2Voltage += 0.01) {
      float out1Voltage = out2Voltage * 2; // Out1 is always 2 times Out2
      
      if (out1Voltage > maxOut1) {
        out1Voltage = maxOut1; // Ensure out1 doesn't exceed max
      }
      
      setVoltage(out1Pin, out1Voltage);
      setVoltage(out2Pin, out2Voltage);
      
      // Debug output
      Serial.print("Up - Out1: ");
      Serial.print(out1Voltage);
      Serial.print("V, Out2: ");
      Serial.print(out2Voltage);
      Serial.println("V");
      
      delay(100);  // Adjust the voltage every 100 ms
    }
    rampingUp = false; // Switch direction
  } else {
    // Ramp down
    for (float out2Voltage = maxOut2; out2Voltage >= idleOut2; out2Voltage -= 0.01) {
      float out1Voltage = out2Voltage * 2; // Out1 is always 2 times Out2
      
      if (out1Voltage > maxOut1) {
        out1Voltage = maxOut1; // Ensure out1 doesn't exceed max
      }
      
      setVoltage(out1Pin, out1Voltage);
      setVoltage(out2Pin, out2Voltage);
      
      // Debug output
      Serial.print("Down - Out1: ");
      Serial.print(out1Voltage);
      Serial.print("V, Out2: ");
      Serial.print(out2Voltage);
      Serial.println("V");
      
      delay(100);  // Adjust the voltage every 100 ms
    }
    rampingUp = true; // Switch direction
  }
}
