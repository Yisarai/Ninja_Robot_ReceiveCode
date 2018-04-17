void RailRunner(){
  Serial.println("RailRunner");
    for (int i=1;i<=10;++i){
    FrontRangeFinder();
    Serial.println(sensorDistance);  
  }
  Serial.print(sensorDistance); 
  while(sensorDistance  < 10){
   FrontRangeFinder();
   Serial.println("Loop 1");
   Serial.println(sensorDistance); ;
   LineFollow();
  }
  LocateLine();
  ResetValues();
  while (sensorDistance > 10){
    Serial.println("Loop 2");
    FrontRangeFinder();
    Serial.println(sensorDistance);
    md.setM4Speed(-250);
    md.setM1Speed(250);
    md.setM2Speed(250);
  }
  ResetValues();
  while (Pos1 < 5){
    Serial.println("Loop 3");
      Serial.println(Pos1);
      Serial.println("Drive Straight");
      PIDControl(5,5);
    }
    Serial.println("Exit");
//    StepperMotor(30);
  }


