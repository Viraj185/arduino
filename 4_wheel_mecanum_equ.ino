
#include <Wire.h>

int8_t up,right,down,left,triangle,circle,cross,squ,lX,lY,rX,rY,touch,vx,vy,W;
double fl,fr,rl,rr,radius;


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
vx=lX;
vy=lY;
W=rX;

fl=(vx-vy-(lx+ly)*W)/radius;
fr=(vx+vy+(lx+ly)*W)/radius;
rl=(vx+vy-(lx+ly)*W)/radius;
rr=(vx-vy+(lx+ly)*W)/radius;
}