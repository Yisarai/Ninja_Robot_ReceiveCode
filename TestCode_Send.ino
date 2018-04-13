#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); // RX, TX

char userInput;

void setup() {
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Open serial communications with the other Arduino board
  mySerial.begin(9600);

  // Call Menu
  CallMenu();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    userInput = Serial.read();
    Serial.print("The user entered ");
    Serial.println(userInput);
    mySerial.write(userInput);
    delay(50);
    switch (userInput) {
      case 'a':
        Serial.println("Front Wheels Running Forward");
        break;

      case 'b':
        Serial.println("Top Wheels Running Forward");
        break;

      case 'c':
        Serial.println("Lead Screw Forward");
        break;

      case 'd':
        Serial.println("Lead Screw Backward");
        break;

      case 'e':
        Serial.println("Arm Down 45 Degrees");
        break;  

      case 'f':
        Serial.println("Line Follow");
        break;

      case 'g':
        Serial.println("Rear Range Finder Value");
        break;

      case 'h':
        Serial.println("Front Range Finder Value");
        break;

      case 'i':
        Serial.println("Hall Effect Sensor Value");
        break;        
      case 'j':
      Serial.println("Shut Down Stepper");
      break;  
   
    }  
    //Wait here until user requests to exit state for robot
    while (Serial.available() == 0) {
      //Do nothing
    }

    // This section sends the order to kick the robot out of whatever state it's in
    userInput = Serial.read();
    mySerial.write(userInput);
    Serial.println();
    CallMenu();
  }
}

//User Defined Functions//
void CallMenu() {
  Serial.println("Functions listed below, please enter the corresponding letter: ");
  Serial.println("a= Front Wheels Running Forward");
  Serial.println("b = Top Wheels Running Forward");
  Serial.println("c = Lead Screw Forward");
  Serial.println("d = Lead Screw Backward");
  Serial.println("e = Arm Down 45 Degrees");  
  Serial.println("f = Line Follow");  
  Serial.println("g = Rear Range Finder Value");  
  Serial.println("h = Front Range Finder Value");  
  Serial.println("i = Hall Effect Sensor Value"); 
  Serial.println("j = Shut Down Stepper"); 
  Serial.println("Caution: Input is case sensitive");
}

