void WarpedWall(){
  Serial.print("Warped Wall");
  ShutDownStepper();
  LineDetect();
  while(sensorDiff > 2500){
    LineFollow();
  }
  BrakeMotor();
  StepperMotor(-20);
  ResetValues();
  while(t_old < 14.5){ //verified
  md.setM4Speed(-400);
  }
  ShutDownStepper();
  delay(1000);
  ResetValues();
  while(t_old < 18){//verified
    md.setM4Speed(400);
  }
  StepperMotor(30);
}

