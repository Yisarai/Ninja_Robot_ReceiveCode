void RailRunner(){
for (int i=1;i<=10;++i){
   FrontRangeFinder();
   Serial.print(sensorDistance); 
 }
  lineLoc = LocateLine();
  ResetValues();
  while (sensorDistance > 10){
    md.setM4Speed(-250);
    md.setM1Speed(250);
    md.setM2Speed(250);
  }
  ResetValues();
  while (Pos1 < 5){
      Serial.println("Drive Straight");
      PIDControl(5,5);
      StepperMotor(30);
    }
  }


