#include <DualVNH5019MotorShieldMod3.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <QTRSensors.h>
#include <Stepper.h>
#include "Encoder.h"

char state = 0;

//-------------------------------------    Hall Effect    -----------------------------------//
const char hallEffectPin = A6;
int raw = 0;

//-------------------------------------    Motor Shield    -----------------------------------//
//Motors 1 and 2 follow the default pins mapping 
//Motor3 - Lead Screw - Negative forward, positive backwards
unsigned char INA3 = 25;
unsigned char INB3 = 26;
unsigned char EN3DIAG3 = 24;
unsigned char PWM3 = 11;
unsigned char CS3 = A2;

//Motor 4 - Top Wheels - Positive Value Runs Backwards
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
const float As = 11.71301;//Side sensor
const float Bs = -1.0421;
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
#define EMITTER_PIN 44
QTRSensorsRC qtrrc((unsigned char[]) {36,37,38,39,40,41,42,43}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
//const int sensorBias[NUM_SENSORS] = {154,117,193,154,183,224,318};
const int sensorBias[NUM_SENSORS] = {417, 298, 377, 337, 299, 336, 417, 583};
unsigned int sensorValues[NUM_SENSORS];
int sensorValueBiased[NUM_SENSORS] = {};
float lineLoc = 0;
int biasSum = 0;
int actualSum = 0;
int sensorDiff = 0;

//-------------------------------------    Stepper Motor    -----------------------------------//
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 46, 47, 48, 49); // negative lowers,positive raises
int stepperCount = 0;              // number of steps the motor has taken
float angle;
int pin1 = 46;
int pin2 = 47;
int pin3 = 48;
int pin4 = 49;

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
double kp1 = 10;
double kd1 = 3.5;
double ki1 = 0;

double kp2 = 10;
double kd2 = 3.5;
double ki2 = 0;

//time variables 
unsigned long t_ms = 0;
double t = 0;                 //current time
double t_old = 0;             //previous time
double deltaT = 0;

//Position variables
double Pos1 = 0;               //current pos
double vel1 = 0;               //Input position
double Pos_old1 = 0;           //previous pos

double Pos2 = 0;               //current pos
double vel2 = 0;               //Input position
double Pos_old2 = 0;           //previous pos

//CONTROL VARIABLES
double error_old1 = 0.0;
double Pos_des1 = 0;
double error1 = 0.0;
double dErrordt1 = 0;
double integralError1 = 0;
int M1 = 0;
float V1 = 0;

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
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
void setup() {
  //Setup pins
  pinMode(frontRangeFinderPin, INPUT);
  pinMode(rearRangeFinderPin, INPUT);
  pinMode(hallEffectPin, INPUT);
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(pin3,OUTPUT);
  pinMode(pin4,OUTPUT);
   myStepper.setSpeed(10);
  // Open serial communications with computer and wait for port to open:
  Serial.begin(9600);
  md.init(); // *changed location
  // Open serial communications with the other Arduino board
  mySerial.begin(9600);

}

//++++++++++++++++++++++++++++++++++++    COMPETITION CODE    +++++++++++++++++++++++++++++++++++++//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
void loop() {
  
  if (mySerial.available()){
    char state = char (mySerial.read());
    Serial.print("The user entered ");
    Serial.println(state);
//    raw = HallEffect();
//    while (raw > 200){          //If this is commented out the switch won't continuously to execute
 
      switch(state){
        case 'a':
          PaddleBoard();
          state='b';
          break;
         
        case 'b':
          WallLift();
          state='c';
          break;
         
        case 'c':
          UTurn();
          state='d';
          break;
         
        case 'd':
          RailRunner();
          state='e';
          break;    
  
        case 'e':
          WarpedWall();
          state='f';
          break; 

        case 'f':
          while (mySerial.available() == 0){
          //Done! Just chill here
          }
          break; 
           
        case 'q':
          while (mySerial.available() == 0){
            FrontRangeFinder();
            Serial.println(sensorDistance);
          }
          break;   
        case 'y':
          md.setM1Speed(100);
          //Right Motor
          md.setM2Speed(100);
          break;  
        case 'x'://backward
          md.setM3Speed(50);
          break;  
        case 'w'://forward
          md.setM3Speed(-100);
          break;  
        case 'z':
          md.setM4Speed(50);
          break;  
        case 's':
          StepperMotor(30);//negative lowers/positive raises
          break;  

      }
  }
//}
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//++++++++++++++++++++++++++++++         CODE DONE          ++++++++++++++++++++++++++++++//


//----------------------------    User Defined Functions     -----------------------------//
//----------------------------------------------------------------------------------------//
void BrakeMotor() {
  md.setM1Brake(0);
  md.setM2Brake(0);
  md.setM3Brake(0);
  md.setM4Brake(0);
}

void ResetValues() {
  //Reset Position
  Pos_des1 = 0;
  Pos_des2 = 0;
  Pos1 = 0;
  Pos2 = 0;
  
  //Reset counts
  counts1 = 0;
  counts2 = 0;
  
  //Reset Error
  error_old1 = 0;
  error_old2 = 0;
  error1 = 0.0;  
  error2 = 0.0;
  dErrordt1 = 0;
  integralError1 = 0;
  dErrordt2 = 0;
  integralError2 = 0;

  //Get Rid of rollover
  t_old = t;  
}

int HallEffect() {
  raw = analogRead(hallEffectPin); //double check pin
  while (raw < 200) {
    raw = analogRead(hallEffectPin);
  }
  return raw;
}

void PIDControl (double vel1, double vel2){
  t_ms = micros();
  t = t_ms/1000000.0;                           //current time 
  
  //Encoder sensing
  counts1 = myEnc1.read();                            //get current counts
  Pos1 = float(counts1)*2*PI/(float(countsPerRev_motor)*GearRatio); //Position in rad
  counts2 = myEnc2.read();                            //get current counts
  Pos2 = float(counts2)*2*PI/(float(countsPerRev_motor)*GearRatio); //Position in rad
  deltaT = t-t_old;  
  
  // --------Position Controller----------------
  // Left Motor
  Pos_des1 = Pos_des1 + vel1*deltaT;
  error1 = Pos_des1 - Pos1;
  dErrordt1 =(error1 - error_old1)/deltaT;
  integralError1 = integralError1 + error1*deltaT;
  V1 = kp1*error1+ kd1*dErrordt1 + ki1*integralError1; 

  // Right Motor
  Pos_des2 = Pos_des2 + vel2*deltaT;
  error2 = Pos_des2 - Pos2;
  dErrordt2 = (error2 - error_old2)/deltaT;
  integralError2 = integralError2 + error2*deltaT;
  V2 = kp2*error2+ kd2*dErrordt2 + ki2*integralError2;
    
  // ---------Motor Command---------------------             
//  V1 = constrain(V1,0,7);   //constrian Motor command to -400 to 400
//  V2 = constrain(V2,0,7);   //constrian Motor command to -400 to 400
  M1 = V1*400.0/9.7;            //convert Voltage to motor command
  M2 = V2*400.0/9.7;            //convert Voltage to motor command
//  Serial.print(M1);
//  Serial.print("\t");
//  Serial.print(M2);
//  Serial.print("\t");
  M1 = constrain(M1,0,400);   //constrian Motor command to -400 to 400
  M2 = constrain(M2,0,400);   //constrian Motor command to -400 to 400
  md.setM1Speed(M1);            //Left Motor Speed
  md.setM2Speed(M2);            //Right Motor Speed

  double velocity1 = (Pos1 - Pos_old1)/deltaT;
  double velocity2 = (Pos2 - Pos_old2)/deltaT;

  Serial.print(t_old);
  Serial.print("\t");

  Serial.print(velocity1);
  Serial.print("\t");
  Serial.print(velocity2);
  Serial.println("\t");

  //save current time and position
  t_old = t;
  Pos_old1 = Pos1;
  error_old1 = error1;
  Pos_old2 = Pos2;
  error_old2 = error2;
}

double LineDetect(){
  qtrrc.read(sensorValues);
  biasSum = 0;
  actualSum = 0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++){
    biasSum = biasSum + sensorBias[i];
    actualSum = actualSum + sensorValues[i];
  }
  sensorDiff = abs(actualSum - biasSum);
  return sensorDiff;
}

double LocateLine() {
  qtrrc.read(sensorValues);
  float num = 0;
  float den = 0;
  for (unsigned char i = 0; i < NUM_SENSORS; i++) { 
    sensorValueBiased[i] = sensorValues[i] - sensorBias[i];
    num = num + sensorValueBiased[i] * i;
    den = den + sensorValueBiased[i];
    
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
  }
  lineLoc = (num/den) - 3.5;
  return lineLoc;
}

void LineFollow(){
  lineLoc = LocateLine();
  if (lineLoc < -0.6) {
    //Left Motor
    md.setM1Speed(20);
    //Right Motor
    md.setM2Speed(80);
  }
  else if(lineLoc < -0.4 & lineLoc >= -0.6){
    //Left Motor
    md.setM1Speed(30);
    //Right Motor
    md.setM2Speed(60);
  }
  else if(lineLoc > 0.4 & lineLoc <= 0.6){
    //Left Motor
    md.setM1Speed(60);
    //Right Motor
    md.setM2Speed(30);
  }
  else if(lineLoc > 0.6){
    //Left Motor
    md.setM1Speed(80);
    //Right Motor
    md.setM2Speed(20);
  }
  else {
    //Left Motor
    md.setM1Speed(45);
    //Right Motor
    md.setM2Speed(45);
  }
  delay(10);
}

double RearRangeFinder(){
  rawData = analogRead(rearRangeFinderPin);
  voltData = 0.0049 * rawData;
  filteredData = voltData * alpha + (1 - alpha) * filteredDataOld;
  filteredDataOld = voltData;
  sensorDistance = Ar * pow(filteredData, Br);
  return sensorDistance;
}

double FrontRangeFinder(){
  rawData = analogRead(frontRangeFinderPin);
  voltData = 0.0049 * rawData;
  filteredData = voltData * alpha + (1 - alpha) * filteredDataOld;
  filteredDataOld = voltData;
  sensorDistance = Af * pow(filteredData, Bf);
  return sensorDistance;
}

void ShutDownStepper(){
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,LOW);
  digitalWrite(pin4,LOW);
}

void StepperMotor(float angle){
//   angle = Serial.parseInt();
   Serial.print("The user entered ");
   Serial.print(angle);
   Serial.println(" degrees");    
   int steps = (int)angle/1.8;
   Serial.print("Steps the arm will move: ");
   Serial.println(steps);
   myStepper.step(steps);
}


