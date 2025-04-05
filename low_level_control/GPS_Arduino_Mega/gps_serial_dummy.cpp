#include <Arduino.h>
float latitude = -6.889852;
float longitude = 107.609968;

void setup() {
  Serial.begin(9600);
}

void loop() {
  char buffer[20];  // Buffer for each coordinate string
  
  Serial.print("Latitude: ");
  // Convert float to string with 6 decimal places
  dtostrf(latitude, 9, 6, buffer);  // width=9, precision=6
  Serial.print(buffer);
  
  Serial.print(", Longitude: ");
  // Convert float to string with 6 decimal places
  dtostrf(longitude, 9, 6, buffer);  // width=9, precision=6
  Serial.println(buffer);
  
  delay(500);
}