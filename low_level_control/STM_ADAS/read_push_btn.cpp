#include <Arduino.h>

// check whether button is pressed or released

void setup() {
    Serial.begin(9600);
    pinMode(2, INPUT_PULLUP); // Set pin 2 as input with pull-up resistor
}

void loop() {
    int buttonState = digitalRead(2); // Read the state of the button
    if (buttonState == LOW) { // Button is pressed
        Serial.println("Button Pressed");
    } else { // Button is released
        Serial.println("Button Released");
    }
    delay(100); // Delay for readability
}