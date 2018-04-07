void RailRunner(){
  FrontRangeFinder();
  LineDetect();
  while (lineLoc < -3)
  {
    md.setM3Speed(250);
    if (sensorDistance < 5)
    {
      Serial.println("Drive Straight");
      PIDControl(100,100,10*PI,10*PI);
    };
  };
}

