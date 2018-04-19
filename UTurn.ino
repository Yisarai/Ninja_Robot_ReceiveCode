void UTurn() {
  Serial.print("U Turn");
  // while line sensor dectects line, run LineFollow code
  StepperMotor(115);               //set stepper angle from ground (-14) to 85 degrees
  delay(1000);
  RearRangeFinder();
  while (sensorDistancer > 6) {
    md.setM1Speed(-50);
    md.setM2Speed(-50);
    RearRangeFinder();
  }  
  ShutDownStepper();
  for (int i=1;i<=10;++i){
   FrontRangeFinder();//sensor is 2.8cm off the ground 
   Serial.print(sensorDistancef); 
 }
  while (sensorDistancef > 2 && sensorDistancef < 10){    // front sensort is above floor or line is detected
    LineFollow();                 // run line follow
    FrontRangeFinder();
    Serial.print(lineLoc);
    Serial.print("\t");
    Serial.println(sensorDistancef);
  }
  BrakeMotor();
}
