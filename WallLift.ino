void WallLift(){
  Serial.print("Wall Lift");
  for (int i=1;i<=10;++i){
    RearRangeFinder();//1.2cm from back, 3.1cm to line follow
    //Serial.println(sensorDistance); 
    Serial.println(raw); //*code put in for testing*//
   
  }
  LineDetect();
  while (sensorDiff > 2500) {//rear sensor wall follow
    LocateLine();
    if (lineLoc < -0.5) {
      md.setM2Speed(-100);
      delay(300);
      BrakeMotor();
      md.setM1Speed(-100);
      delay(415);
      BrakeMotor();
      md.setM2Speed(70);
      md.setM1Speed(70);
      delay(500);
      BrakeMotor();
    }
    else if(lineLoc > 0.5){
      md.setM1Speed(-100);
      delay(300);
      BrakeMotor();
      md.setM2Speed(-100);
      delay(400);
      BrakeMotor();
      md.setM2Speed(70);
      md.setM1Speed(70);
      delay(500);
      BrakeMotor();
    }
    else{
      md.setM1Speed(325);
      md.setM2Speed(325);
      md.setM4Speed(250);
      delay(1500);
      StepperMotor(20);
      delay(500);
      StepperMotor(20);
      delay(500);
      ShutDownStepper();
    }
    LineDetect();
  }
  while (sensorDistance > 10) {
    RearRangeFinder();
  }
  BrakeMotor();  
}
