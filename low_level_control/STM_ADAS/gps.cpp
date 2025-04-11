// // // // #include <Arduino.h>
// // // // #include <TinyGPS++.h>
// // // // #include <HardwareSerial.h>

// // // // // Create a GPS object
// // // // TinyGPSPlus gps;

// // // // // // Hardware serial port for GPS module
// // // // // HardwareSerial Serial1(USART1);

// // // // void setup() {
// // // //   Serial.begin(9600);       // Initialize Serial Monitor
// // // //   Serial2.begin(9600);      // Initialize GPS module on Serial1
// // // //   Serial.println("Initialize");
// // // // }

// // // // void loop() {
// // // //   // Check if data is available on Serial1
// // // //   while (Serial2.available() > 0) {
// // // //     char c = Serial2.read();  // Read the incoming byte
// // // //     if (gps.encode(c)) {      // Parse the GPS data
// // // //       // Print latitude and longitude if available
// // // //       if (gps.location.isUpdated()) {
// // // //         Serial.print("Latitude: ");
// // // //         Serial.print(gps.location.lat(), 6);
// // // //         Serial.print(", Longitude: ");
// // // //         Serial.println(gps.location.lng(), 6);
// // // //       }
// // // //     }
// // // //   }
// // // // }

// // // #include <Arduino.h>
// // // #include <TinyGPS++.h>

// // // // Create a GPS object
// // // TinyGPSPlus gps;

// // // // Define the hardware serial for STM32 (check your board's specific UART ports)
// // // HardwareSerial mySerial(1);  // Use UART1, adjust the port number if necessary

// // // void setup() {
// // //   Serial.begin(9600);         // Initialize Serial Monitor (for debugging)
// // //   mySerial.begin(9600);       // Initialize GPS module on UART1
// // //   Serial.println("Initialize");
// // // }

// // // void loop() {
// // //   // Check if data is available on the GPS serial port (mySerial)
// // //   while (mySerial.available() > 0) {
// // //     char c = mySerial.read();  // Read the incoming byte
// // //     if (gps.encode(c)) {      // Parse the GPS data
// // //       // Print latitude and longitude if available
// // //       if (gps.location.isUpdated()) {
// // //         Serial.print("Latitude: ");
// // //         Serial.print(gps.location.lat(), 6);
// // //         Serial.print(", Longitude: ");
// // //         Serial.println(gps.location.lng(), 6);
// // //       }
// // //     }
// // //   }
// // //   // Serial.println("Initialize");
// // // }

#include <SoftwareSerial.h>

#include <TinyGPS++.h>

/* This sample code demonstrates the normal use of a TinyGPS object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/

TinyGPS gps;
SoftwareSerial ss(4, 3);

void setup()
{
  Serial.begin(115200);
  ss.begin(4800);
  
  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
}

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}

