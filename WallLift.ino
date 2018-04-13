void WallLift(){ 
  for (int i=1;i<=10;++i){
    sensorDistance = RearRangeFinder();     //1.2cm from back, 3.1cm to line follow 
  }
  while (sensorDistance > 5) {              //rear sensor wall follow
    md.setM1Speed(400);
    md.setM2Speed(400);  
    md.setM4Speed(100);
    sensorDistance = RearRangeFinder();
  }
  BrakeMotor();   
}

