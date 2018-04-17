void UTurn() {
  Serial.print("U Turn");
  // while line sensor dectects line, run LineFollow code
  StepperMotor(115);               //set stepper angle from ground (-14) to 85 degrees
  delay(1000);
  ShutDownStepper();
  for (int i=1;i<=10;++i){
   FrontRangeFinder();//sensor is 2.8cm off the ground 
   Serial.print(sensorDistance); 
 }
  while (sensorDistance > 2 && sensorDistance < 10){    // front sensort is above floor or line is detected
    LineFollow();                 // run line follow
    FrontRangeFinder();
    Serial.print(lineLoc);
    Serial.print("\t");
    Serial.println(sensorDistance);
  }
  BrakeMotor();
}
