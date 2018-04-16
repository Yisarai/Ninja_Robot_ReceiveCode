void WallLift(){
for (int i=1;i<=10;++i){
 RearRangeFinder();//1.2cm from back, 3.1cm to line follow
 Serial.print(sensorDistance); 
}
while (sensorDistance > 4) {//rear sensor wall follow
LocateLine();
RearRangeFinder();
 if (lineLoc < -0.5) {
   md.setM2Speed(-100);
   delay(300);
   BrakeMotor();
   md.setM1Speed(-100);
   delay(500);
   BrakeMotor();
   md.setM2Speed(70);
   md.setM1Speed(70);
   delay(500);
   BrakeMotor();
 }
 else if(lineLoc > 0.5){
   md.setM1Speed(-100);
   delay(300);
   BrakeMotor();
   md.setM2Speed(-100);
   delay(500);
   BrakeMotor();
   md.setM2Speed(70);
   md.setM1Speed(70);
   delay(500);
   BrakeMotor();
 }
// else if (lineLoc < -1.5) {
//   md.setM2Speed(-100);
//   delay(500);
//   BrakeMotor();
//   md.setM1Speed(-100);
//   delay(650);
//   BrakeMotor();
//   md.setM2Speed(70);
//   md.setM1Speed(70);
//   delay(700);
//   BrakeMotor();
// }
// else if(lineLoc > 1.5){
//   md.setM1Speed(-100);
//   delay(500);
//   BrakeMotor();
//   md.setM2Speed(-100);
//   delay(650);
//   BrakeMotor();
//   md.setM2Speed(70);
//   md.setM1Speed(70);
//   delay(700);
//   BrakeMotor();
// }
 else{
   md.setM1Speed(325);
   md.setM2Speed(325);
   md.setM4Speed(250);
   delay(1000);
   StepperMotor(20);
   delay(1000);
   ShutDownStepper();
   delay(5000);
   
 }
}
BrakeMotor();  
}
