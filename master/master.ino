#include <Wire.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
Wire.beginTransmission(9);
Wire.write(0);
Wire.endTransmission();
}
//e8:47:3a:5a:a3:66
