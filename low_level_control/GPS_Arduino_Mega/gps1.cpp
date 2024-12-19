#include <Arduino.h>
#include <TinyGPS++.h>

// Create a GPS object
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);       // Initialize Serial Monitor
  Serial1.begin(9600);      // Initialize GPS module on Serial1
  Serial.println("Initialize");
}

void loop() {
  // Check if data is available on Serial1
  while (Serial1.available() > 0) {
    char c = Serial1.read();  // Read the incoming byte
    if (gps.encode(c)) {      // Parse the GPS data
      // Print latitude and longitude if available
      if (gps.location.isUpdated()) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", Longitude: ");
        Serial.println(gps.location.lng(), 6);
      }
    }
  }
}