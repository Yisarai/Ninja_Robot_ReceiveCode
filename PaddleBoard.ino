void PaddleBoard(){
  StepperMotor(-96);
  delay(3000);
  ShutDownStepper();
  while (lineLoc < -2){
     LineDetect();
     PIDControl (2,2);
   }
}
