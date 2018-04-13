void RailRunner(){
  for (int i=1;i<=10;++i){
    sensorDistance = FrontRangeFinder();
    Serial.println(sensorDistance);
  }
  sensorDistance = LineDetect();
  lineLoc = LocateLine();
  Pos1=0;
  while (Pos1 < 25){
    md.setM3Speed(250);
    md.setM1Speed(250);
    md.setM2Speed(250);
    if (sensorDistance < 10){
      Serial.println("Drive Straight");
      PIDControl(2,2);
      StepperMotor(30);
    }
  }
}

