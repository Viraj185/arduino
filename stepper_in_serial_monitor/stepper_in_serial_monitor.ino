#define dir 9
#define rev 5
int steps;
int angle;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
pinMode(rev,OUTPUT);
pinMode(dir,OUTPUT);
}

void loop() {
  
  // put your main code here, to run repeatedly:
 if (Serial.available()>0){
    angle=Serial.parseInt();
    Serial.println(angle);
  }
steps=map(angle,0,360,0,800);

digitalWrite(dir,HIGH);
 for(int i=0 ; i<steps ; i++){
  
  digitalWrite(rev,HIGH);
  delayMicroseconds(500);
  
  digitalWrite(rev,LOW);
  delayMicroseconds(500);
 }
}



 
  
 
 
 
  
 
  
  
