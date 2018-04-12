void WallLift(){
   
   RearRangeFinder();
   StepperMotor(-104);
   while (sensorDistance > 5) {//rear sensor wall follow
   
   md.setM1Speed(350);
   md.setM2Speed(-350);
   md.setM4Speed(-100);
 }
}

