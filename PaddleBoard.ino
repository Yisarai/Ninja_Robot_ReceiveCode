void PaddleBoard(){
  StepperMotor(-105);
  delay(1000);
  ShutDownStepper();
  LineDetect();
  while (abs(biasSum-actualSum) < 1500){
     PIDControl (2,2);
     LineDetect();
   }
   BrakeMotor();
}
