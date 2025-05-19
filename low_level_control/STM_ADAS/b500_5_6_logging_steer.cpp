#include <Arduino.h>

// Definisi pin untuk ADC
#define ANALOG1 A0
#define ANALOG2 A1
#define POTENTIOMETER_PIN PB0

// Definisi pin untuk output DAC
#define DAC1 PA4
#define DAC2 PA5

// Mode Kontrol
bool manualMode = true; // Default: Manual Mode

// Nilai idle state 2.5V pada resolusi 10-bit (2.5V / 3.3V * 1023)
#define IDLE_STATE 775

// Variabel tegangan untuk mode keyboard
float voltage1 = 2.5;
float voltage2 = 2.5;

// ADC Constants for steering
const int ADC_MAX = 1023;            // 10-bit ADC max value
const int ADC_CENTER = 512;          // Center position (adjusted for 10-bit)

// Scaling factors for steering angle
const float degrees_per_adc_ccw = 90.0 / 40.0; // 2.25 deg/ADC for counterclockwise
const float degrees_per_adc_cw = 90.0 / 40.0;  // 2.25 deg/ADC for clockwise

// IIR Filter parameters
const float Wc = 2.8629; // Cutoff frequency in rad/s
const float Ts = 0.1;    // Sampling period in seconds

// Dynamically computed IIR filter coefficients
float b0, b1, b2, a1, a2;

// Buffers for previous inputs and outputs
float prev_input_1 = 0.0;
float prev_input_2 = 0.0;
float filtered_value = 0.0;
float prev_filtered_1 = 0.0;
float prev_filtered_2 = 0.0;

// Variabel untuk smoothing di manual mode
static float smoothed_val1 = IDLE_STATE;
static float smoothed_val2 = IDLE_STATE;
const float alpha = 0.05; // Faktor smoothing (0–1, kecil = lebih halus)

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        delay(10);
    }
    
    // Initialize pins
    pinMode(POTENTIOMETER_PIN, INPUT);
    analogReadResolution(10); // Mengatur resolusi ADC ke 10-bit
    analogWriteResolution(10); // Mengatur resolusi DAC ke 10-bit
    
    // Set DAC ke idle state awal
    analogWrite(DAC1, IDLE_STATE);
    analogWrite(DAC2, IDLE_STATE);
    
    // Calculate IIR filter coefficients
    float alpha = (Wc * Ts) / 2.0;
    float denom = 1.0 + sqrt(2.0) * alpha + alpha * alpha;
    a1 = -2.0 * (1.0 - alpha * alpha) / denom;
    a2 = (1.0 - sqrt(2.0) * alpha + alpha * alpha) / denom;
    b0 = (alpha * alpha) / denom;
    b1 = 2.0 * b0;
    b2 = b0;
    
    // Initialize filter with first reading
    float initial_value = analogRead(POTENTIOMETER_PIN);
    prev_input_1 = initial_value;
    prev_input_2 = initial_value;
    filtered_value = initial_value;
    prev_filtered_1 = filtered_value;
    prev_filtered_2 = filtered_value;
    
    Serial.println("Steering Control System Initialized");
    Serial.println("Mode: Manual (default)");
    Serial.println("Tekan 'm' untuk beralih mode, 'a'/'d' untuk kontrol keyboard");
}

void loop() {
    // Handle serial input for mode and keyboard control
    if (Serial.available()) {
        char input = Serial.read();
        
        if (input == 'm' || input == 'M') {
            manualMode = !manualMode;
            if (!manualMode) {
                voltage1 = 2.5;
                voltage2 = 2.5;
                analogWrite(DAC1, (voltage1 / 3.3) * 1023);
                analogWrite(DAC2, (voltage2 / 3.3) * 1023);
            }
            Serial.print("Mode: ");
            Serial.println(manualMode ? "Manual" : "Keyboard");
        }
        
        if (!manualMode) {
            if (input == 'a' || input == 'A') {
                if (voltage1 < 3.3 && voltage2 > 0.0) {
                    voltage1 += 0.1;
                    voltage2 -= 0.1;
                }
            } else if (input == 'd' || input == 'D') {
                if (voltage1 > 0.0 && voltage2 < 3.3) {
                    voltage1 -= 0.1;
                    voltage2 += 0.1;
                }
            }
            // Set DAC dalam mode keyboard (10-bit)
            analogWrite(DAC1, (voltage1 / 3.3) * 1023);
            analogWrite(DAC2, (voltage2 / 3.3) * 1023);
            
            Serial.print("DAC Output - PA4: ");
            Serial.print(voltage1, 2);
            Serial.print("V, PA5: ");
            Serial.print(voltage2, 2);
            Serial.println("V");
        }
    }
    
    // Read potentiometer for steering angle
    float rawValue = analogRead(POTENTIOMETER_PIN);
    
    // Apply 2nd-order IIR filter
    filtered_value = b0 * rawValue + b1 * prev_input_1 + b2 * prev_input_2 - a1 * prev_filtered_1 - a2 * prev_filtered_2;
    
    // Update previous values
    prev_input_2 = prev_input_1;
    prev_input_1 = rawValue;
    prev_filtered_2 = prev_filtered_1;
    prev_filtered_1 = filtered_value;
    
    // Calculate steering angle
    float deviation = ADC_CENTER - filtered_value;
    float steeringAngle;
    if (deviation > 0) {
        steeringAngle = deviation * degrees_per_adc_ccw;
    } else {
        steeringAngle = deviation * degrees_per_adc_cw;
    }
    
    // Clamp steering angle to ±720 degrees
    if (steeringAngle > 720.0) steeringAngle = 720.0;
    if (steeringAngle < -720.0) steeringAngle = -720.0;
    
    // Handle DAC in manual mode
    if (manualMode) {
        int val1 = analogRead(ANALOG1);
        int val2 = analogRead(ANALOG2);
        
        // Terapkan low-pass filter
        smoothed_val1 = (1 - alpha) * smoothed_val1 + alpha * val1;
        smoothed_val2 = (1 - alpha) * smoothed_val2 + alpha * val2;
        
        // Tulis ke DAC
        analogWrite(DAC1, val1 > 0 ? (int)smoothed_val1 : IDLE_STATE);
        analogWrite(DAC2, val2 > 0 ? (int)smoothed_val2 : IDLE_STATE);
        voltage1 = (smoothed_val1 / 1023.0) * 3.3;
        voltage2 = (smoothed_val2 / 1023.0) * 3.3;
    }
    
    // Log data in CSV format: time_ms,voltage1,voltage2,mode,raw_value,filtered_value,steering_angle
    unsigned long time_ms = millis();
    Serial.print(time_ms);
    Serial.print(",");
    Serial.print(voltage1, 2);
    Serial.print(",");
    Serial.print(voltage2, 2);
    Serial.print(",");
    Serial.print(manualMode ? "Manual" : "Keyboard");
    Serial.print(",");
    Serial.print(rawValue);
    Serial.print(",");
    Serial.print(filtered_value);
    Serial.print(",");
    Serial.print(steeringAngle, 1);
    Serial.println();
    
    delay(10); // Controlled logging rate
}