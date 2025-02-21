#define interrupt1 2
#define interrupt2 3
int spd=5;
int dir=9;
int input=A0;

volatile long change;
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 75.00;

void setup()
{
  pinMode(interrupt1,INPUT_PULLUP);
  pinMode(interrupt2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt1), blink1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt2), blink2, CHANGE);
  pinMode(5,OUTPUT);
  pinMode(9,OUTPUT);
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;
  last_time = 0;
  Serial.begin(9600);
  
  for(int i = 0; i < 50; i++)
  {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
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


void loop()
{
  double now = millis();
  dt = (now - last_time)/1000.00;
  last_time = now;

  double actual = map(analogRead(input), 0, 1024, 0, 255);
  double error = setpoint - actual;
  output = pid(error);

  analogWrite(dir, output);

  // Setpoint VS Actual
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);

  // Error
  //Serial.println(error);

  delay(300);
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
