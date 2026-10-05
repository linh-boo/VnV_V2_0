// #include <Arduino.h>
// #include <Wire.h>
// #include "pin_mapping.h"


// void setup() {
//   Serial.begin(115200);
//   Wire.begin(PIN_SDA, PIN_SCL);  // ESP32 default pins or specify: Wire.begin(SDA, SCL);

//   Serial.println("\nI2C Scanner");
// }

// void loop() {
//   byte error, address;
//   int count = 0;

//   Serial.println("Scanning...");

//   for (address = 1; address < 127; address++) {
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0) {
//       Serial.print("Found device at 0x");
//       if (address < 16) Serial.print("0");
//       Serial.println(address, HEX);
//       count++;
//     } else if (error == 4) {
//       Serial.print("Unknown error at 0x");
//       Serial.println(address, HEX);
//     }
//   }

//   if (count == 0) {
//     Serial.println("No I2C devices found\n");
//   } else {
//     Serial.println("Done\n");
//   }

//   delay(3000); // scan every 3 seconds
// }