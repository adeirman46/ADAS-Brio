#include <Arduino.h>

// analog read for sensor angle reading

void setup() {
    Serial.begin(9600);       // Inisialisasi komunikasi serial
    pinMode(A0, INPUT);
}

void loop() {
    int angle = analogRead(A0);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(100);
}

