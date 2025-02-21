const int pinA = 2;
const int pinB = 3;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int stateA = digitalRead(pinA);
  int stateB = digitalRead(pinB);
  
  Serial.print("State A: ");
  Serial.print(stateA);
  Serial.print(" State B: ");
  Serial.println(stateB);
  delay(1000);
}
