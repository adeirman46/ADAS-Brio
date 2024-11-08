#include <Arduino.h>
#include <Servo.h>  

Servo myServo;  
int brakeValue = 90;  // Start at fully released position (90 degrees)
const int brakeStep = 8;  // 10% of range (80 degrees total range)
const int MIN_ANGLE = 90;   // Fully released
const int MAX_ANGLE = 180;  // Fully pressed
const int ANGLE_RANGE = MAX_ANGLE - MIN_ANGLE;

void setup() {
    Serial.begin(9600);       
    myServo.attach(D5);      
    myServo.write(MIN_ANGLE);  // Initial position - fully released
    Serial.println("Brake Control Ready:");
    Serial.println("b - Increase brake (move toward 180°)");
    Serial.println("n - Decrease brake (move toward 90°)");
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        
        switch(command) {
            case 'b':  // Increase brake (move toward 170)
                if (brakeValue <= (MAX_ANGLE - brakeStep)) {
                    brakeValue += brakeStep;  // Increase angle to increase brake
                    myServo.write(brakeValue);
                    Serial.print("Brake at: ");
                    Serial.print(((brakeValue - MIN_ANGLE) * 100) / ANGLE_RANGE);  // Calculate percentage
                    Serial.println("%");
                }
                break;
                
            case 'n':  // Decrease brake (move toward 90)
                if (brakeValue >= (MIN_ANGLE + brakeStep)) {
                    brakeValue -= brakeStep;  // Decrease angle to decrease brake
                    myServo.write(brakeValue);
                    Serial.print("Brake at: ");
                    Serial.print(((brakeValue - MIN_ANGLE) * 100) / ANGLE_RANGE);  // Calculate percentage
                    Serial.println("%");
                }
                break;
        }
    }
}