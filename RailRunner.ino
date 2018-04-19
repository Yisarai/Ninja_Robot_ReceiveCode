void RailRunner(){
  Serial.println("RailRunner");
    for (int i=1;i<=10;++i){
    FrontRangeFinder();
    Serial.println(sensorDistancef);  
  }
  LineDetect();
  Serial.print(sensorDistancef); 
  while(sensorDistancef < 10 && sensorDiff > 2500){
   FrontRangeFinder();
   LineDetect();
   Serial.println("Loop 1");
   Serial.println(sensorDistancef); ;
   LineFollow();
  }
  while (sensorDistancef > 10){
    Serial.println("Loop 2");
    FrontRangeFinder();
    LineDetect();
    Serial.println(sensorDistancef);
    md.setM3Speed(-250);
    md.setM1Speed(250);
    md.setM2Speed(250);
  }
  if(sensorDistancef > 2 && sensorDistancef < 10){
  StepperMotor(30);
    Serial.println("Loop 3");
    FrontRangeFinder();
  }
  delay(5000);
  LineDetect();
  while (sensorDistancef > 2 && sensorDistancef < 10 && sensorDiff < 2500){
    Serial.println("Loop 4");
    FrontRangeFinder();
    LineDetect();
    Serial.println(sensorDistancef);
    md.setM3Speed(-250);
    md.setM1Speed(250);
    md.setM2Speed(250);
  }
  BrakeMotor();
}


