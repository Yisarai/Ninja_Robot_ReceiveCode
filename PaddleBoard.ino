void PaddleBoard(){
  StepperMotor(-115);
  delay(1000);
  ShutDownStepper();

  for (int i=1;i<=10;++i){
    sensorDistance = FrontRangeFinder();
  }
  ResetValues();   
//  while (sensorDistance < 15);{     //This is for when we attach the side range finder.
    while (Pos1 < 50){
      PIDControl (2,2);
    }
//  }
   BrakeMotor();
}
