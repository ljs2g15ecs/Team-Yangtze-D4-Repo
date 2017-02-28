#include <Wire.h>

byte x = 0;

void setup() {
  Wire.begin(1); // join i2c bus (address optional for master)
}

void loop() {
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write("X is: ");        // sends five bytes
  Wire.write(x);              // sends one byte
  Wire.endTransmission();    // stop transmitting

  x++;
  delay(500);
}
