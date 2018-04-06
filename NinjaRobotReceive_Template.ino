#include <DualVNH5019MotorShieldMod3.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <QTRSensors.h>
#include <Stepper.h>
#include "Encoder.h"
int state = 1;
int raw = 0;
const int NOFIELD = 412;

//Motor 1
unsigned char INA1 = 22;
unsigned char INB1 = 23;
unsigned char EN1DIAG1 = 24;
unsigned char PWM1 =9;
unsigned char CS1 = A0;

//Motor 2
unsigned char INA2 = 25;
unsigned char INB2 = 26;
unsigned char EN2DIAG2 = 27;
unsigned char PWM2 = 10;
unsigned char CS2 = A1;
//Motor 3
unsigned char INA3 = 44;
unsigned char INB3 = 45;
unsigned char EN3DIAG3 = 46;
unsigned char PWM3 = 50;
unsigned char CS3 = A2;

//Motor 4
unsigned char INA4 = 47;
unsigned char INB4 = 48;
unsigned char EN4DIAG4 = 49;
unsigned char PWM4 = 51;
unsigned char CS4 = A3;

//DualVNH5019MotorShieldMod3 md;
DualVNH5019MotorShieldMod3 md = DualVNH5019MotorShieldMod3 (INA1, INB1, EN1DIAG1, CS1,INA2, INB2, EN2DIAG2, CS2,INA3, INB3, EN3DIAG3, CS3,INA4, INB4, EN4DIAG4, CS4, PWM1, PWM2, PWM3,PWM4);

SoftwareSerial mySerial(52, 53); // RX, TX
void setup() {
    Serial.begin(9600);
  md.init(); // *changed location
  // Open serial communications with the other Arduino board
  mySerial.begin(9600);

}

void loop() {
while (mySerial.available()== 0){
HallEffect();
}
while (raw =< 200){
switch(state){
  case '1':
   while (mySerial.available() == 0) {
          PaddleBoard();
        }
        state=2;
   break;
}
}
}
//User Defined Functions//
void PControl (){
}//need to add in
void BreakMotor() {
  md.setM1Brake(0);
  md.setM2Brake(0);
  md.setM3Brake(0);
  md.setM4Brake(0);
  void HallEffect() {
  //  Uncomment this to get a raw reading for calibration of no-field point
  //  Serial.print("Raw reading: ");
  //  Serial.println(raw);
  int raw = analogRead(A4); //double check pin

  while (raw > 1000) {
    raw = analogRead(A4);
    md.setM1Speed(150);
    md.setM2Speed(-150);
  }
   BreakMotor();
}
}

