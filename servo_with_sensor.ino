
int sensor;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
pinMode(A1,INPUT);
pinMode(12,OUTPUT);
pinMode(6,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(12,HIGH);
  
sensor=analogRead(A1);
Serial.println(sensor);
if(sensor<=240){
  analogWrite(6,255);
}
else{
  analogWrite(6,0);
}
}
