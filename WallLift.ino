void WallLift(){
  Serial.print("Wall Lift");
  for (int i=1;i<=10;++i){
    RearRangeFinder();//1.2cm from back, 3.1cm to line follow
//    Serial.println(sensorDistance); 
//    Serial.println(raw); //*code put in for testing*//
   
  }
  LineDetect();
  while (sensorDiff > 4000 ) {//rear sensor wall follow
    lineLoc = LocateLine();
    M = PID(0, lineLoc, 60, Kd, Ki);

    if (lineLoc < 0.1){
      ML_Final = 50 + M;
      MR_Final = 50 - M;      
    }
    else if (lineLoc > -0.1){
      ML_Final = 50 - M;
      MR_Final = 50 + M;      
    }
    else {
      ML_Final = 75;
      MR_Final = 75;   
    }
    //Left Motor
    md.setM1Speed(ML_Final);
    //Right Motor
    md.setM2Speed(MR_Final);
    
//    if (lineLoc < -0.6) {
//      //Left Motor
//      md.setM1Speed(20);
//      //Right Motor
//      md.setM2Speed(80);
//    }
//    else if(lineLoc < -0.4 & lineLoc >= -0.6){
//      //Left Motor
//      md.setM1Speed(30);
//      //Right Motor
//      md.setM2Speed(60);
//    }
//    else if(lineLoc > 0.4 & lineLoc <= 0.6){
//      //Left Motor
//      md.setM1Speed(60);
//      //Right Motor
//      md.setM2Speed(30);
//    }
//    else if(lineLoc > 0.6){
//      //Left Motor
//      md.setM1Speed(80);
//      //Right Motor
//      md.setM2Speed(20);
//    }
//    else {
//      //Left Motor
//      md.setM1Speed(45);
//      //Right Motor
//      md.setM2Speed(45);
//    }
//    delay(10);    
//    LocateLine();
//    if (lineLoc < -0.5) {
//      md.setM2Speed(-100);
//      delay(300);
//      BrakeMotor();
//      md.setM1Speed(-100);
//      delay(465);
//      BrakeMotor();
//      md.setM2Speed(70);
//      md.setM1Speed(70);
//      delay(500);
//      BrakeMotor();
//    }
//    else if(lineLoc > 0.5){
//      md.setM1Speed(-100);
//      delay(300);
//      BrakeMotor();
//      md.setM2Speed(-100);
//      delay(520);
//      BrakeMotor();
//      md.setM2Speed(70);
//      md.setM1Speed(70);
//      delay(500);
//      BrakeMotor();
//    }
//    else{
//      md.setM1Speed(50);
//      md.setM2Speed(50);
//      md.setM4Speed(-200);
//      for (int i=1;i<=20;++i){
//        md.setSpeeds(400, 400, 200, 0);
//      }
//      delay(2500);
//      StepperMotor(20);
//      delay(500);
//      ShutDownStepper();      
//      StepperMotor(20);
//      delay(500);
//      ShutDownStepper();      
//    }
    LineDetect();
  }
  md.setSpeeds(400, 400, 200, 0);
  delay(2000);
  StepperMotor(20);
  delay(500);
  ShutDownStepper();      
  StepperMotor(20);
  delay(500);
  ShutDownStepper(); 
  while (sensorDistancer > 10) {
    RearRangeFinder();
  }
  BrakeMotor();  
}
