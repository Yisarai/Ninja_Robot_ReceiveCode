#include <DualVNH5019MotorShieldMod3.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <QTRSensors.h>
#include <Stepper.h>
char userInput;
//Hall Effect
const char hallEffectPin = A6;
int raw = 0;
const int NOFIELD = 412;

//Motor 3
unsigned char INA3 = 25;
unsigned char INB3 = 26;
unsigned char EN3DIAG3 = 24;
unsigned char PWM3 = 11;
unsigned char CS3 = A2;

//Motor 4
unsigned char INA4 = 29;
unsigned char INB4 = 30;
unsigned char EN4DIAG4 = 28;
unsigned char PWM4 = 5;
unsigned char CS4 = A3;

//Stepper
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 46, 47, 48, 49);
int stepCount = 0;         // number of steps the motor has taken
float angle;
int pin1 = 46;
int pin2 = 47;
int pin3 = 48;
int pin4 = 49;

//RangeFinder
const float Af = 11.30109925;//rear sensor
const float Bf = -0.9211;
const float Ar = 11.71301;//right sensor
const float Br = -1.0421;
const float alpha = 0.1;
const char frontRangeFinderPin = A7;
const char rearRangeFinderPin = A5;

float filteredDataOld = 0;
float filteredData = 0;
int rawData = 0;
float sensorDistance = 0;
float voltData = 0;

//IR Sensor
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 3
QTRSensorsRC qtrrc((unsigned char[]) {36,37,38,39,40,41,42,43}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
const int sensorBias[NUM_SENSORS] = {154,117,193,154,183,224,318};
unsigned int sensorValues[NUM_SENSORS];
int sensorValueBiased[NUM_SENSORS] = {};
float lineLoc = -4;
int biasSum = 0;
int actualSum = 0;
int sensorDiff = 0;

//Encoder
#include "Encoder.h"
Encoder myEnc1(20,21);          //create an instance of Encoder
Encoder myEnc2(18,19);          //create an instance of Encoder
double GearRatio = 131;         //the gear ratio
double rWheel = 0.035;          //Wheel Radius
int countsPerRev_motor = 64;    //the counts per revolution of the motor shaft
long counts1 = 0;               //Globally initialize encoder counts
long counts2 = 0;               //Globally initialize encoder counts

//DualVNH5019MotorShieldMod3 md;
DualVNH5019MotorShieldMod3 md = DualVNH5019MotorShieldMod3 (INA3, INB3, EN3DIAG3, CS3, PWM3, INA4, INB4, EN4DIAG4, CS4,PWM4);

SoftwareSerial mySerial(52, 53); // RX, TX

void setup() {
  pinMode(frontRangeFinderPin, INPUT);
  pinMode(rearRangeFinderPin, INPUT);
  pinMode(hallEffectPin, INPUT);
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(pin3,OUTPUT);
  pinMode(pin4,OUTPUT);
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);
  md.init(); // *changed location
  // Open serial communications with the other Arduino board
  mySerial.begin(9600);
}
void loop() {
  // put your setup code here, to run once:
  if (mySerial.available()) {
    char userInput = char (mySerial.read());
    Serial.print("The user entered ");
    Serial.println(userInput);
    delay(50);


    switch (userInput) {

      case 'a':
      Serial.println("Front Wheels Running Forward");
        while (mySerial.available() == 0) {
          //Left Motor
          md.setM1Speed(100);
          //Right Motor
          md.setM2Speed(100);
        }
        break;
        
      case 'b':
      Serial.println("Top Wheels Running Forward");
        while (mySerial.available() == 0) {
          //Top Motor
          md.setM4Speed(-150);
        }
        break;

      case 'c':
      Serial.print("Lead Screw Forward");
        while (mySerial.available() == 0) {
          //Lead Screw Forward
          md.setM3Speed(-50);
        }
        break;

      case 'd':
      Serial.print("Lead Screw Backward");
        while (mySerial.available() == 0) {
          //Lead Screw Backward
          md.setM3Speed(50);
        }
        break;
        
      case 'e':
      Serial.print("Arm Down 45 Degrees");
        while (mySerial.available() == 0) {
          //Stepper Motor, Arm Down 
          StepperMotor(-45);
        }
        break;
        
      case 'f':
      Serial.print("Line Follow");
        while (mySerial.available() == 0) {
          //QTR Sensor
          LineFollow();
        }
        break;
      case 'g':
      while (mySerial.available() == 0) {
        Serial.print("Rear Range Finder Value");
        Serial.print("\t");
          //Rear Range Finder
          RearRangeFinder();
        Serial.println(sensorDistance);
        }
        break;
      case 'h':
        while (mySerial.available() == 0) {
          Serial.print("Front Range Finder Value");
          Serial.print("\t");
          //Front Range Finder
          FrontRangeFinder();
          Serial.println(sensorDistance);
        }
        break;

      case 'i':
        while (mySerial.available() == 0) {
          Serial.print("Hall Effect Sensor Value");
          //Hall Effect Sensor 
          HallEffect();
          Serial.println(raw);
        }
        break;
      case 'j':
      Serial.print("Shut Down Stepper");
        while (mySerial.available() == 0) {
          //Stepper Motor, Arm Down 
          ShutDownStepper();
        }
        break;
    }
    BrakeMotor();
}
}


//User Defined Functions//
void BrakeMotor() {
  md.setM1Brake(0);
  md.setM2Brake(0);
  md.setM3Brake(0);
  md.setM4Brake(0);
}

void HallEffect() {
  raw = analogRead(hallEffectPin); //double check pin
  while (raw < 200) {
    raw = analogRead(hallEffectPin);
  }
}

void StepperMotor(float angle){
   Serial.print("The user entered ");
   Serial.print(angle);
   Serial.println(" degrees");    
   int steps = (int)angle/1.8;
   Serial.print("Steps the arm will move: ");
   Serial.println(steps);
   myStepper.step(steps);
}
void ShutDownStepper(){
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,LOW);
  digitalWrite(pin4,LOW);

}
void RearRangeFinder(){
  rawData = analogRead(rearRangeFinderPin);
  voltData = 0.0049 * rawData;
  filteredData = voltData * alpha + (1 - alpha) * filteredDataOld;
  filteredDataOld = voltData;
  sensorDistance = Ar * pow(filteredData, Br);
}

void FrontRangeFinder(){
  rawData = analogRead(frontRangeFinderPin);
  voltData = 0.0049 * rawData;
  filteredData = voltData * alpha + (1 - alpha) * filteredDataOld;
  filteredDataOld = voltData;
  sensorDistance = Af * pow(filteredData, Bf);
}
void LineDetect() {
    qtrrc.read(sensorValues);
    float num = 0;
    float den = 0;

    for (unsigned char i = 0; i < NUM_SENSORS; i++) {

    sensorValueBiased[i] = sensorValues[i] - sensorBias[i];
    num = num + sensorValueBiased[i] * i;
    den = den + sensorValueBiased[i];

    Serial.print(sensorValues[i]);
    Serial.print('\t');
  };
  lineLoc = num / den - 4.5;
}
void LineFollow(){
  LineDetect();
  Serial.println(lineLoc);
  if (lineLoc < - 0.5 && lineLoc > -3) {
    Serial.println("Turn left");
    //Left Motor
    md.setM1Speed(100);
    //Right Motor
    md.setM2Speed(0);
  }
  else if (lineLoc > 0.5) {
    Serial.println("Turn Right");

       //Right Motor
    md.setM2Speed(-100);
    //Left Motor
    md.setM1Speed(0);
  }
  else if(lineLoc <= -3){
  BrakeMotor();
  }
  else {
    Serial.println("Drive Straight");
    //Left Motor
    md.setM1Speed(45);
    //Right Motor
    md.setM2Speed(-45);
  }
  delay(10);
}

