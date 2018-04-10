void WallLift(){   
  RearRangeFinder();
  while (sensorDistance > 5) {//rear sensor wall follow
    PIDControl (6,6);
    md.setM4Speed(100);
    RearRangeFinder();
  }
  BrakeMotor();   
}

