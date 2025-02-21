#define interrupt1 2
#define interrupt2 3
int spd=5;
int dir=9;
volatile long change;
long counter=0;
int lastcounter=0;
int aState=0;
int LastState=0;
int d;
int starttime;
int stoptime=500;
int c;

void setup() {
  pinMode(interrupt1,INPUT_PULLUP);
  pinMode(interrupt2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt1), blink1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt2), blink2, CHANGE);
  Serial.begin(9600);
  pinMode(5,OUTPUT);
  pinMode(9,OUTPUT);
   starttime=millis();
}

void blink1() {
  noInterrupts();
  int stateA = digitalRead(interrupt1);  
  int stateB = digitalRead(interrupt2);  
  
  
  if (stateA == stateB) {
    change++;  
  } 
  else {
    change--;  
  }
  interrupts();

}

void blink2() {
  noInterrupts();
  int stateA = digitalRead(interrupt1);  
  int stateB = digitalRead(interrupt2);  
  
  
  if (stateA != stateB) {
    change++;  
  } 
  else {
    change--;  
  }
  interrupts();

}

void loop() {
 
  digitalWrite(dir, HIGH);     
  analogWrite(spd, 100);         

  d = change * 0.058; 

 


  
   if (d > 500) {
    analogWrite(spd, 0);
 
  } 
  
    Serial.print("d: ");
    Serial.println(d);
  
  
 
}
