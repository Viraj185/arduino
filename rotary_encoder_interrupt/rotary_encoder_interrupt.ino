#define interrupt1 2
#define interrupt2 3
volatile long change;
long counter=0;
int lastcounter=0;
int aState=0;
int LastState=0;
int d;

void setup() {
  pinMode(interrupt1,INPUT_PULLUP);
  pinMode(interrupt2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt1), blink1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt2), blink2, CHANGE);
  Serial.begin(9600);
}

void blink1() {
  int stateA = digitalRead(interrupt1);  
  int stateB = digitalRead(interrupt2);  
  
  
  if (stateA == stateB) {
    change++;  
  } 
  else {
    change--;  
  }
}

void blink2() {
  int stateA = digitalRead(interrupt1);  
  int stateB = digitalRead(interrupt2);  
  
  
  if (stateA != stateB) {
    change++;  
  } 
  else {
    change--;  
  }
}

void loop() {
  
  if(change != counter){
    Serial.println(change);
    counter=change;
  }
  
  
}





