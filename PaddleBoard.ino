void PaddleBoard(){
  StepperMotor(-115);
  delay(1000);
  ShutDownStepper();
  Pos1 = 0;
  t_old = t;
  while (Pos1 < 25){
    PIDControl (PI/12,PI/12);
  }
  sensorDiff = LineDetect();
  lineLoc = LocateLine();  
  while (sensorDiff < 3000){
    sensorDiff = LineDetect();
    lineLoc = LocateLine();  
  }
   BrakeMotor();
}
