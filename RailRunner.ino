void RailRunner(){
  FrontRangeFinder();
  LineDetect();
  LocateLine();
  while (sensorDiff < 2500 && lineLoc <= -1)
  {
    md.setM3Speed(250);
    if (sensorDistance < 10)
    {
      Serial.println("Drive Straight");
      PIDControl(2,2);
    };
  };
}

