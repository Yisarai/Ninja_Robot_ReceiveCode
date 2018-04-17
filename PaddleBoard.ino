void PaddleBoard(){
  Serial.print("PaddleBoard");
  StepperMotor(-115);
  delay(1000);
  ShutDownStepper();
  for (int i=1;i<=10;++i){
    SideRangeFinder();
    Serial.println(sensorDistance);  
  }
  ResetValues();  
  Serial.print(sensorDistance); 
  while (sensorDistance < 15){     //This is for when we attach the side range finder.
    PIDControl (4,4);
    SideRangeFinder();
  }
  BrakeMotor();
}
