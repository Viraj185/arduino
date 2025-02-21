int spd=5;
int dir=9;
int meter;
int motor;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(5,OUTPUT);
pinMode(9,OUTPUT);
pinMode(A1,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir,HIGH);
  meter=analogRead(A1);
  Serial.println(meter);
  motor=map(meter,0,1023,0,100);
  analogWrite(spd,motor);
}
