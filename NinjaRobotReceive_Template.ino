#include <DualVNH5019MotorShieldMod3.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <QTRSensors.h>
#include <Stepper.h>
#include "Encoder.h"

int state = 0;

//-------------------------------------    Hall Effect    -----------------------------------//
const char hallEffectPin = A6;
int raw = 0;

//-------------------------------------    Motor Shield    -----------------------------------//
//Motors 1 and 2 follow the default pins mapping 
//Motor3
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

DualVNH5019MotorShieldMod3 md = DualVNH5019MotorShieldMod3 (INA3, INB3, EN3DIAG3, CS3, PWM3, INA4, INB4, EN4DIAG4, CS4, PWM4);

//-------------------------------------    Range Finder    -----------------------------------//
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

//-------------------------------------    IR Sensor    -----------------------------------//
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 3
QTRSensorsRC qtrrc((unsigned char[]) {36,37,38,39,40,41,42,43}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
const int sensorBias[NUM_SENSORS] = {237,178,178,237,237,237,299,362};
unsigned int sensorValues[NUM_SENSORS];
int sensorValueBiased[NUM_SENSORS] = {};

//-------------------------------------    Stepper Motor    -----------------------------------//
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 46, 47, 48, 49);
int stepperCount = 0;              // number of steps the motor has taken

//-------------------------------------    PID Control    -----------------------------------//
//Encoder and motor globals
Encoder myEnc1(20,21);          //create an instance of Encoder
Encoder myEnc2(18,19);          //create an instance of Encoder
double GearRatio = 131;         //the gear ratio
double rWheel = 0.035;          //Wheel Radius
int countsPerRev_motor = 64;    //the counts per revolution of the motor shaft
long counts1 = 0;               //Globally initialize encoder counts
long counts2 = 0;               //Globally initialize encoder counts

//Gains
double kd1 = 0;
double kp1 = 2;
double ki1 = 0;
double kd2 = 0;
double kp2 = 2.23;
double ki2 = 0;
double f = 0.5;                 //frequency in Hz

//time variables 
unsigned long t_ms = 0;
double t = 0;                 //current time
double t_old = 0;             //previous time

double Pos1 = 0;               //current pos
double Vel1 = 0;               //current velocity
double Pos_old1 = 0;           //previous pos

double Pos2 = 0;               //current pos
double Vel2 = 0;               //current velocity
double Pos_old2 = 0;           //previous pos

//CONTROL VARIABLES
double Pos_des_old1 = 0;           
double error_old1 = 0.0;
double Pos_des1 = 0;
double error1 = 0.0;
double dErrordt1 = 0;
double integralError1 = 0;
int M1 = 0;
float V1 = 0;

double Pos_des_old2 = 0;           
double error_old2 = 0.0;
double Pos_des2 = 0;
double error2 = 0.0;
double dErrordt2 = 0;
double integralError2 = 0;
int M2 = 0;
float V2 = 0;

//Distance
double distanceDes = 0;
double distance1 = 0;
double distance2 = 0;

//-------------------------------------    Communication    -----------------------------------//
SoftwareSerial mySerial(52, 53); // RX, TX

//+++++++++++++++++++++++++++++++++++++    COMPETITION SETUP    ++++++++++++++++++++++++++++++++++//
void setup() {
  //Setup pins
  pinMode(frontRangeFinderPin, INPUT);
  pinMode(rearRangeFinderPin, INPUT);
  pinMode(hallEffectPin, INPUT);
  
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);
  md.init(); // *changed location
  // Open serial communications with the other Arduino board
  mySerial.begin(9600);

}

//++++++++++++++++++++++++++++++++++++    COMPETITION CODE    +++++++++++++++++++++++++++++++++++++//
void loop() {
  if (mySerial.available()){
    state = mySerial.read();
    HallEffect();
    while (raw > 200){
    qtrrc.read(sensorValues);
    float num = 0;
    float den = 0;

    for (unsigned char i = 0; i < NUM_SENSORS; i++) {

    sensorValueBiased[i] = sensorValues[i] - sensorBias[i];
    num = num + sensorValueBiased[i] * i;
    den = den + sensorValueBiased[i];

    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  float lineLoc = num / den - 4.5;
  
      switch(state){
        case '1':
          PaddleBoard();
          state=2;
          break;
         
        case '2':
          WallLift();
          state=3;
          break;
         
        case '3':
          UTurn();
          state=4;
          break;
         
        case '4':
          RailRunner();
          state=5;
          break;    
  
        case '5':
          WarpedWall();
          state=6;
          break;  

        case '6':
          while (mySerial.available() == 0){
          //Done! Just chill here
          }
          break;
                                   
      }
    }
  }
}

//----------------------------    User Defined Functions     -----------------------------//

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

void PIDControl (double Vel1,double Vel2,double Pos_des1,double Pos_des2){
  t_ms = micros();
  t = t_ms/1000000.0;                           //current time 

  //Encoder sensing
  counts1 = myEnc1.read();                            //get current counts
  Pos1 = float(counts1)*2*PI/(float(countsPerRev_motor)*GearRatio); //Position in rad
  counts2 = myEnc2.read();                            //get current counts
  Pos2 = float(counts2)*2*PI/(float(countsPerRev_motor)*GearRatio); //Position in rad
    
  // --------Position Controller----------------
  // Left Motor
  Pos_des1 = Pos_des1 + Vel1*(t-t_old);
  dErrordt1 = ((Pos_des1 - Pos1)-(Pos_des_old1 - Pos_old1))/(t-t_old);
  error1 = Pos_des1 - Pos1;
  integralError1 = integralError1 + error1*(t-t_old);
  V1 = kp1*(Pos_des1 - Pos1)+ kd1*dErrordt1 + ki1*integralError1;

  // Right Motor
  Pos_des2 = Pos_des2 + Vel2*(t-t_old);
  dErrordt2 = ((Pos_des2 - Pos2)-(Pos_des_old2 - Pos_old2))/(t-t_old);
  error2 = Pos_des2 - Pos2;
  integralError2 = integralError2 + error2*(t-t_old);
  V2 = kp2*(Pos_des2 - Pos2)+ kd2*dErrordt1 + ki2*integralError2;
  
  // ---------Motor Command---------------------                        
  M1 = V1*400.0/9.7;            //convert Voltage to motor command
  M2 = V2*400.0/9.7;            //convert Voltage to motor command
  M1 = constrain(M1,-400,400);   //constrian Motor command to -400 to 400
  M2 = constrain(M2,-400,400);   //constrian Motor command to -400 to 400
  md.setM1Speed(M1);            //Left Motor Speed
  md.setM1Speed(M2);            //Right Motor Speed

  //save current time and position
  t_old = t;
  Pos_old1 = Pos1;
  Pos_des_old1 = Pos_des1;
  error_old1 = error1;
  Pos_old2 = Pos2;
  Pos_des_old2 = Pos_des2;
  error_old2 = error2;
}

void LineFollow(){
//  qtrrc.read(sensorValues);
//  float num = 0;
//  float den = 0;
//
//  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
//
//    sensorValueBiased[i] = sensorValues[i] - sensorBias[i];
//    num = num + sensorValueBiased[i] * i;
//    den = den + sensorValueBiased[i];
//
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }
//  float lineLoc = num / den - 4.5;
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
  else if(lineLoc <= -3)
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
void StepperMotor(float angle){
   if (Serial.available()) {
   float angle = Serial.parseInt();
   Serial.print("The user entered ");
   Serial.print(angle);
   Serial.println(" degrees");    
   int steps = (int)angle/1.8;
   Serial.print("Steps the arm will move: ");
   Serial.println(steps);
   myStepper.step(-steps);
 }
}

