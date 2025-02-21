int timee;
int distance;
void setup() {
  // put your setup code here, to run once:
pinMode(8,OUTPUT);
pinMode(9,INPUT);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
delay(2);
digitalWrite(8,HIGH);
delay(10);
digitalWrite(8,LOW);

timee=pulseIn(9,HIGH);
distance=timee*0.034/2;

Serial.print(distance);
Serial.println("cm");
}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
