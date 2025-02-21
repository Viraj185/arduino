#define dir 9
#define rev 5
int potentio;
int steps;
int oldAngle=0;
int newAngle=0;
int newpot;
int Osteps=0;
int Nsteps;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
pinMode(rev,OUTPUT);
pinMode(dir,OUTPUT);
pinMode(A0,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
potentio=analogRead(A0);
Nsteps=Nsteps-Osteps;
Nsteps=map(potentio,0,1023,0,800);
newAngle=map(Nsteps,0,800,0,360);
//Serial.println(Nsteps);
  if(newAngle-oldAngle>0) {
    digitalWrite(dir,HIGH);
   Nsteps=map(potentio,0,1023,0,800);
   newAngle=map(Nsteps,0,800,0,360);
   steps=Nsteps-Osteps;

    for(int i=0 ; i<steps ; i++){
      
      digitalWrite(rev,HIGH);
      delayMicroseconds(500);
      
      digitalWrite(rev,LOW);
      delayMicroseconds(500);
    }
  }
    if(newAngle-oldAngle<0) {
    digitalWrite(dir,LOW);
   Nsteps=map(potentio,0,1023,0,800);
   newAngle=map(Nsteps,0,800,0,-360);
   steps=Nsteps-Osteps;
   steps=steps*(-1);
   Serial.println(newAngle);
    for(int i=0 ; i<steps ; i++){
      
      digitalWrite(rev,HIGH);
      delayMicroseconds(500);
      
      digitalWrite(rev,LOW);
      delayMicroseconds(500);
    }
  }
oldAngle=newAngle;
Osteps=Nsteps;
Serial.println(newAngle);
delay(100);

}
