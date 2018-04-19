void PaddleBoard(){
Serial.println("PaddleBoard");
  StepperMotor(-115);
  delay(1000);
  ShutDownStepper();
  for (int i=1;i<=10;++i){
    SideRangeFinder();
  //  Serial.println(sensorDistances);
  }
  ResetValues();
  while (sensorDistances < 20){
    WallFollow(10.5);
    SideRangeFinder();
   // Serial.println(sensorDistancea);
  }
}
