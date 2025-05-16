#include <Arduino.h>

const int out1Pin = PB10; // DAC1
const int out2Pin = PB4; // DAC2

// Target voltages for idle and max states
const float idleOut1 = 1.0;  // 1V
const float idleOut2 = 0.5;  // 0.5V
const float maxOut1 = 2.0;   // 2V
const float maxOut2 = 1.0;   // 1V

// Voltage range for DAC on STM32F446RE is 0 to 3.3V
const float dacMaxVoltage = 3.3;
const int dacResolution = 4095; // 12-bit resolution (0-4095)

// Current output voltages
float currentOut1 = idleOut1;
float currentOut2 = idleOut2;

// Helper function to set the output voltage on a DAC pin
void setVoltage(int dacPin, float voltage) {
  // Scale the voltage to DAC value (12-bit resolution)
  int dacValue = (voltage / dacMaxVoltage) * dacResolution;
  
  // Ensure the DAC value stays within bounds
  dacValue = constrain(dacValue, 0, dacResolution);
  
  // Use analogWrite instead of dacWrite for STM32
  analogWrite(dacPin, dacValue);
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  
  // Initialize DAC
  analogWriteResolution(12); // Set 12-bit resolution
  
  // Set initial idle state
  setVoltage(out1Pin, currentOut1);
  setVoltage(out2Pin, currentOut2);
  
  // Serial.println("Holding idle state for 15 seconds...");
  // delay(15000);  // Hold idle state for 15 seconds
}

void loop() {
  // Check if any data has been received from Serial
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'w') {
      // Increase voltage
      if (currentOut2 < maxOut2) {
        currentOut2 += 0.1;
        currentOut1 = currentOut2 * 2; // Out1 is always 2 times Out2
        if (currentOut1 > maxOut1) {
          currentOut1 = maxOut1;
        }
        
        setVoltage(out1Pin, currentOut1);
        setVoltage(out2Pin, currentOut2);
        
        // Debug output
        Serial.print("Increased - Out1: ");
        Serial.print(currentOut1);
        Serial.print("V, Out2: ");
        Serial.print(currentOut2);
        Serial.println("V");
      } else {
        Serial.println("Maximum voltage reached");
      }
      
    } else if (command == 's') {
      // Decrease voltage
      if (currentOut2 > idleOut2) {
        currentOut2 -= 0.1;
        currentOut1 = currentOut2 * 2; // Out1 is always 2 times Out2
        if (currentOut1 < idleOut1) {
          currentOut1 = idleOut1;
        }
        
        setVoltage(out1Pin, currentOut1);
        setVoltage(out2Pin, currentOut2);
        
        // Debug output
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
  
  delay(100);  // Small delay for smoother control response
}
