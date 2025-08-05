#include <Wire.h>
#include <ps5Controller.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.begin();
ps5.begin("7c:66:ef:78:76:f0");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (ps5.isConnected()) {
 Wire.beginTransmission(9);
 Wire.write(ps5.Up());
 Wire.write(ps5.Right());
 Wire.write(ps5.Down());
 Wire.write(ps5.Left());
 Wire.write(ps5.Triangle());
 Wire.write(ps5.Circle());
 Wire.write(ps5.Cross());
 Wire.write(ps5.Square());
 Wire.write(ps5.LStickX());
 Wire.write(ps5.LStickY());
 Wire.write(ps5.RStickY());
 Wire.write(ps5.RStickY());
 Wire.write(ps5.Touchpad());

 Wire.endTransmission();


  }
 

}
//R1 e8:47:3a:5a:a3:66
//R2 7c:66:ef:78:76:f0

