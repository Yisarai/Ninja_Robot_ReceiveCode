#include <SoftwareSerial.h>
char userInput;

SoftwareSerial mySerial(2,3); // RX, TX

void setup() {
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);

  // Open serial communications with the other Arduino board
  mySerial.begin(9600);
}
 void loop (){
   if (Serial.available()) {
    userInput = Serial.read();
    Serial.print("The user entered ");
    Serial.println(userInput);
    mySerial.write(Serial.read());
    delay(50);
 }
 }

