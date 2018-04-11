void PaddleBoard(){
  StepperMotor(-115);
  delay(1000);
  ShutDownStepper();
  LineDetect();
  LocateLine();
  while (mySerial.available() == 0){
//  while (sensorDiff < 3000 || lineLoc <= -1){
     PIDControl (2,2);
     LineDetect();
     LocateLine();
   }
   BrakeMotor();
}
