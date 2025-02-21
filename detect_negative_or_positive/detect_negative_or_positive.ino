void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect (necessary for native USB devices)
  }
  S****erial.println("Enter a number: ");
}

void loop() {
  // put your main code here, to run repeatedly:
 if (Serial.available() > 0) {
    // Read the incoming number
    int num = Serial.parseInt();
    
    // Check if the number is positive or negative
    if (num > 0) {
      Serial.print("The number ");
      Serial.print(num);
      Serial.println(" is positive.");
    }
    else if (num < 0) {
      Serial.print("The number ");
      Serial.print(num);
      Serial.println(" is negative.");
    }
    else {
      Serial.println("The number is zero.");
    }
    
    // Wait a moment before asking for another input
    delay(1000);
    Serial.println("Enter a number: ");
  }

}
