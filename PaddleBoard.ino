void PaddleBoard(){
  StepperMotor(-115);
  delay(1000);
  ShutDownStepper();
  LineDetect();
  LocateLine();
  t_old=millis()/1000;
  while (sensorDiff < 3000 || lineLoc <= -1 || Pos1 < 25){
     PIDControl (2,2);
     LineDetect();
     LocateLine();
   }
   BrakeMotor();
}
