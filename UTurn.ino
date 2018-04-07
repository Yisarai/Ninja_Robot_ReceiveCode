void UTurn() {
  // while line sensort dectects line, run LineFollow code

FrontRangeFinder();
while (sensorDistance =<5 || lineLoc >= -3 ){   // front sensort is above floor or line is detected

StepperMotor(96);   //set stepper angle from ground (-14) to 85 degrees

 LineFollow();   // run line follow
 
  }
}
