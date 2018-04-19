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
const int noMag = 650;
int absMag = 0;

//-------------------------------------    Motor Shield    -----------------------------------//
//Motors 1 and 2 follow the default pins mapping 
//Motor3 - Top Wheels - Negative forward, positive backwards
unsigned char INA3 = 25;
unsigned char INB3 = 26;
unsigned char EN3DIAG3 = 24;
unsigned char PWM3 = 11;
unsigned char CS3 = A2;

//Motor 4 - Lead Screw - Positive Value Runs Backwards
unsigned char INA4 = 29;
unsigned char INB4 = 30;
unsigned char EN4DIAG4 = 28;
unsigned char PWM4 = 5;
unsigned char CS4 = A3;

DualVNH5019MotorShieldMod3 md = DualVNH5019MotorShieldMod3 (INA3, INB3, EN3DIAG3, CS3, PWM3, INA4, INB4, EN4DIAG4, CS4, PWM4);

//-------------------------------------    Range Finder    -----------------------------------//
const float Af = 5.7293;// new sensor (front)
const float Bf = -1.5986;
const float Ar = 11.98913;// rear sensor
const float Br = -1.0649;
const float Ari = 11.71301;//right sensor
const float Bri = -1.0421;
const float Aa = 11.3010992;//arm sensor
const float Ba = -0.9211;
const float alpha = 0.1;
const char frontRangeFinderPin = A7;
const char rearRangeFinderPin = A5;
const char sideRangeFinderPin = A8;
const char armRangeFinderPin = A9;

float distanceDesired = 0;

float filteredDataOldr = 0;
float filteredDatar = 0;
int rawDatar = 0;
float sensorDistancer = 0;
float voltDatar = 0;

float filteredDataOldf = 0;
float filteredDataf = 0;
int rawDataf = 0;
float sensorDistancef = 0;
float voltDataf = 0;

float filteredDataOlds = 0;
float filteredDatas = 0;
int rawDatas = 0;
float sensorDistances = 0;
float voltDatas = 0;

float filteredDataOlda = 0;
float filteredDataa = 0;
int rawDataa = 0;
float sensorDistancea = 0;
float voltDataa = 0;

//-------------------------------------    IR Sensor    -----------------------------------//
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 44
QTRSensorsRC qtrrc((unsigned char[]) {36,37,38,39,40,41,42,43}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
const int sensorBias[NUM_SENSORS] = {154,117,193,154,183,224,318};//course
//const int sensorBias[NUM_SENSORS] = {417, 298, 377, 337, 299, 336, 417, 583};
unsigned int sensorValues[NUM_SENSORS];
int sensorValueBiased[NUM_SENSORS] = {};
float lineLoc = 0;
int biasSum = 0;
int actualSum = 0; 
int sensorDiff = 0;
int actualSumLeft = 0;
int biasSumLeft = 0;
int sensorDiffLeft = 0;
int actualSumRight = 0;
int biasSumRight = 0; 
int sensorDiffRight = 0;

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
double Kp = 80;
double Kd = 2.25;
double Ki = 0.25;

double kp1 = 8;
double kd1 = 2;
double ki1 = 1;

double kp2 = 8.78;
double kd2 = 2;
double ki2 = 1;

//time variables 
unsigned long t_ms = 0;
double t = 0;                 //current time
double t_old = 0;             //previous time
unsigned long t_ref = 0; 
double deltaT = 0;

//Position variables
double Pos = 0;               //current pos
double vel = 0;               //Input position
double Pos_old = 0;           //previous pos

double Pos1 = 0;               //current pos
double vel1 = 0;               //Input position
double Pos_old1 = 0;           //previous pos

double Pos2 = 0;               //current pos
double vel2 = 0;               //Input position
double Pos_old2 = 0;           //previous pos

//CONTROL VARIABLES
double error_old = 0.0;
double Pos_des = 0;
double error = 0.0;
double dErrordt = 0;
double integralError = 0;
double M = 0;
double MR_Final = 0;
double ML_Final = 0;

float V = 0;

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
//    HallEffect();
  absMag = 301;
    while (absMag > 300){          //If this is commented out the switch won't continuously to execute
 
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
            ArmRangeFinder();
            Serial.println(sensorDistancea);
          }
          
          break;   
        case 'r':
          while (mySerial.available() == 0){        
          HallEffect();
          }
          break;
          
        case 'y':
          md.setM1Speed(100);
          //Right Motor
          md.setM2Speed(100);
          break;  
        case 'x'://backward
          md.setM4Speed(100);
          break;  
        case 'w'://forward
          md.setM4Speed(-400);
          break;  
        case 'z':
          md.setM3Speed(-30);
          break;  
        case 's':
          StepperMotor(30);//negative lowers/positive raises
          break;  

      }
  }
  }
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
  t_ref = micros(); 
}

void HallEffect() {
  raw = analogRead(hallEffectPin); //double check pin
  absMag = abs(raw-noMag);
  while (absMag < 300) {
    raw = analogRead(hallEffectPin);
    Serial.println(raw);
    absMag = abs(raw-noMag);
  }
}

void PIDControl (double vel1, double vel2){
  t_ms = micros()-t_ref;
  t = t_ms/1000000.00;                           //current time 
 
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
  M1 = constrain(M1,-400,400);   //constrian Motor command to -400 to 400
  M2 = constrain(M2,-400,400);   //constrian Motor command to -400 to 400
  md.setM1Speed(M1);            //Left Motor Speed
  md.setM2Speed(M2);            //Right Motor Speed

  double velocity1 = (Pos1 - Pos_old1)/deltaT;
  double velocity2 = (Pos2 - Pos_old2)/deltaT;

//  Serial.print(t);
//  Serial.print("\t");
//
//  Serial.print(error1);
//  Serial.print("\t");
//  Serial.print(error2);
//  Serial.println("\t");

  //save current time and position
  t_old = t;
  Pos_old1 = Pos1;
  error_old1 = error1;
  Pos_old2 = Pos2;
  error_old2 = error2;
}

void LineDetect(){
  qtrrc.read(sensorValues);
  biasSum = 0;
  actualSum = 0;
  actualSumLeft = 0;
  biasSumLeft = 0;
  actualSumRight = 0;
  biasSumRight = 0;    
  for (unsigned char i = 0; i < NUM_SENSORS; i++){
    biasSum = biasSum + sensorBias[i];
    actualSum = actualSum + sensorValues[i];
  }
  sensorDiff = abs(actualSum - biasSum);
  for (unsigned char i = 0; i < 2; i++){
    biasSumLeft = biasSumLeft + sensorBias[i];
    actualSumLeft = actualSumLeft + sensorValues[i];
  }  
  sensorDiffLeft = abs(actualSumLeft - biasSumLeft);
  for (unsigned char i = 6; i < NUM_SENSORS; i++){
    biasSumRight = biasSumRight + sensorBias[i];
    actualSumRight = actualSumRight + sensorValues[i];
  }
  sensorDiffRight = abs(actualSumRight - biasSumRight);
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
  lineLoc = (num/den) - 4.5;

  return lineLoc;
}

void LineFollow(){
  lineLoc = LocateLine();
  float thresh = 2.7;
  float c = 1/thresh;
  float P = c*lineLoc;
  float basespeed = 50;
  float RMS = basespeed*(1-P);
  float LMS = basespeed*(1+P);
  md.setM1Speed(LMS);
  md.setM2Speed(RMS);
//  if (lineLoc < -0.6) {
//    //Left Motor
//    md.setM1Speed(20);
//    //Right Motor
//    md.setM2Speed(80);
//  }
//  else if(lineLoc < -0.4 & lineLoc >= -0.6){
//    //Left Motor
//    md.setM1Speed(30);
//    //Right Motor
//    md.setM2Speed(60);
//  }
//  else if(lineLoc > 0.4 & lineLoc <= 0.6){
//    //Left Motor
//    md.setM1Speed(60);
//    //Right Motor
//    md.setM2Speed(30);
//  }
//  else if(lineLoc > 0.6){
//    //Left Motor
//    md.setM1Speed(80);
//    //Right Motor
//    md.setM2Speed(20);
//  }
//  else {
//    //Left Motor
//    md.setM1Speed(45);
//    //Right Motor
//    md.setM2Speed(45);
//  }
//  delay(10);

}

void RearRangeFinder(){
  rawDatar = analogRead(rearRangeFinderPin);
  voltDatar = 0.0049 * rawDatar;
  filteredDatar = voltDatar * alpha + (1 - alpha) * filteredDataOldr;
  filteredDataOldr = voltDatar;
  sensorDistancer = Ar * pow(filteredDatar, Br);
}

void FrontRangeFinder(){
  rawDataf = analogRead(frontRangeFinderPin);
  voltDataf = 0.0049 * rawDataf;
  filteredDataf = voltDataf * alpha + (1 - alpha) * filteredDataOldf;
  filteredDataOldf = voltDataf;
  sensorDistancef = Af * pow(filteredDataf, Bf);
}

void SideRangeFinder(){
  rawDatas = analogRead(sideRangeFinderPin);
  voltDatas = 0.0049 * rawDatas;
  filteredDatas = voltDatas * alpha + (1 - alpha) * filteredDataOlds;
  filteredDataOlds = voltDatas;
  sensorDistances = Ari * pow(filteredDatas, Bri);
}

void ArmRangeFinder(){
  rawDataa = analogRead(armRangeFinderPin);
  voltDataa = 0.0049 * rawDataa;
  filteredDataa = voltDataa * alpha + (1 - alpha) * filteredDataOlda;
  filteredDataOlda = voltDataa;
  sensorDistancea = Aa * pow(filteredDataa, Ba);
}

void ShutDownStepper(){
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,LOW);
  digitalWrite(pin4,LOW);
}

void WallFollow(double distanceDesired) {
  rawDataa = analogRead(armRangeFinderPin);
  voltDataa = 0.0049 * rawDataa;
  filteredDataa = voltDataa * alpha + (1 - alpha) * filteredDataOlda;
  filteredDataOlda = voltDataa;
  sensorDistancea = Aa * pow(filteredDataa, Ba);
//  Serial.print("Sensor distance is ");
//  Serial.println(sensorDistancea);
    if (sensorDistancea > distanceDesired-1 && sensorDistancea < distanceDesired+1) {//arm sensor wall follow
    //Right Motor
    md.setM2Speed(150);
    //Left Motor
    md.setM1Speed(150);
  }
  else if (sensorDistancea > distanceDesired+1 && sensorDistancea < 15){
    //Right Motor
    md.setM2Speed(150);
    //Left Motor
    md.setM1Speed(100);
  }
  else if (sensorDistancea < distanceDesired-1){
    //Right Motor
    md.setM2Speed(100);
    //Left Motor
    md.setM1Speed(150); 
  }
  else if (sensorDistancea >= 15){
    //Right Motor
    md.setM2Speed(100);
    //Left Motor
    md.setM1Speed(115);
  }
}

void StepperMotor(float angle){
//   angle = Serial.parseInt();
//   Serial.print("The user entered ");
//   Serial.print(angle);
//   Serial.println(" degrees");    
   int steps = (int)angle/1.8;
//   Serial.print("Steps the arm will move: ");
//   Serial.println(steps);
   myStepper.step(steps);
}

double PID(int Pos_des, int Pos, int Kp, int Kd, int Ki) {
  t_ms = micros()-t_ref;
  t = t_ms/1000000.00;                           //current time 
  deltaT = t-t_old;  

  //QTR sensing

  deltaT = t-t_old;  

  // --------Position Controller----------------
  error = Pos_des - Pos;
  dErrordt = (error - error_old)/deltaT;
  integralError = integralError + error*deltaT;
  M = Kp*error+ Kd*dErrordt + Ki*integralError;
  M = constrain(M,-100,100);   //constrian Motor command to -400 to 400

  return M;
}


