#include <Wire.h>

int8_t up,right,down,left,triangle,circle,cross,squ,lX,lY,rX,rY,touch;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Wire.begin(9);
Wire.onReceive(receiveEvents);
}
void receiveEvents(int value){
 up = Wire.read();
 right = Wire.read();
 down = Wire.read();
 left = Wire.read();
 triangle = Wire.read();
 circle = Wire.read();
 cross = Wire.read();
 squ = Wire.read();
 lX = Wire.read();
 lY = Wire.read();
 rX = Wire.read();
 rY = Wire.read();
 touch = Wire.read();

}
void loop() {
  // put your main code here, to run repeatedly:
Serial.print(lX);
Serial.print(" ");
Serial.print(lY);
Serial.print(" ");
Serial.print(rX);
Serial.print(" ");
Serial.print(rY);
Serial.print(" ");
Serial.print(up);
Serial.print(" ");
Serial.print(down);
Serial.print(" ");
Serial.print(right);
Serial.print(" ");
Serial.print(left);
Serial.print(" ");
Serial.print(touch);
Serial.print(" ");
Serial.print(triangle);
Serial.print(" ");
Serial.print(cross);
Serial.print(" ");
Serial.print(circle);
Serial.print(" ");
Serial.print(squ);
Serial.println(" ");


if(up==1){
  Serial.println("UP");
}
else if(right==1){
  Serial.println("Right");
}
else if(down==1){
  Serial.println("Down");
  }
else if(left==1){
  Serial.println("Left");
}
else if(triangle==1){
  Serial.println("TRIANGLE");
}
else if(circle==1){
  Serial.println("CIRCLE");
}
else if(cross==1){
  Serial.println("CROSS");
}
else if(squ==1){
  Serial.println("SQUARE");
}
else if(touch==1){
  Serial.println("Touch");
}
Serial.println(lX);
Serial.println(lY);
Serial.println(rX);
Serial.println(rY);
}
