#include <Arduino.h>
#include <Servo.h>  // Mengimpor library Servo

Servo myServo;  // Membuat objek servo

void setup() {
  Serial.begin(9600);       // Inisialisasi komunikasi serial
  myServo.attach(D5);       // Menghubungkan servo ke pin D5 (ubah sesuai kebutuhan)
  myServo.write(0);         // Mengatur posisi awal servo ke 0 derajat
  Serial.println("Masukkan sudut antara 0 hingga 180 derajat:");
}

void loop() {
  // if (Serial.available() > 0) {  // Mengecek apakah ada input dari serial
  //   String input = Serial.readStringUntil('\n');  // Membaca input hingga newline
  //   int angle = input.toInt();                    // Mengonversi input ke integer

  //   // Mengecek apakah sudut berada dalam rentang yang valid
  //   if (angle >= 0 && angle <= 180) {
  //     myServo.write(angle);                       // Menggerakkan servo ke sudut yang ditentukan
  //     Serial.print("Servo bergerak ke ");
  //     Serial.print(angle);
  //     Serial.println(" derajat");
  //   } else {
  //     Serial.println("Input tidak valid. Masukkan sudut antara 0 hingga 180 derajat.");
  //   }
  // }
  myServo.write(90);
  delay(1000);
  myServo.write(0);
  delay(1000);
}
