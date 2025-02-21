#define outA 2
#define outB 3
int counter=0;
int aState;
int Laststate;

void setup() {
  // put your setup code here, to run once:
pinMode(outA,INPUT);
pinMode(outB,INPUT);
pinMode(11,OUTPUT);
digitalWrite(11,HIGH);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
aState=digitalRead(outA);
  if(aState != Laststate){
      if(digitalRead(outB) != aState){
        counter++;
      }
      else{
        counter--;
      }
    Serial.print("Position : ") ; 
    Serial.println(counter);
  }
Laststate=aState;
}
