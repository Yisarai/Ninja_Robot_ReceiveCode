void WallLift(){   
  RearRangeFinder();//1.2cm from back, 3.1cm to line follow 
  while (sensorDistance > 5) {//rear sensor wall follow
    PIDControl (6,6);
    md.setM4Speed(100);
    RearRangeFinder();
  }
  BrakeMotor();   
}

