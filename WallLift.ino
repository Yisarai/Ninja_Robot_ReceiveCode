void WallLift(){ 
  for (int i=1;i<=10;++i){
   RearRangeFinder();//1.2cm from back, 3.1cm to line follow 
 }
  t_old=millis()/1000;
  while (sensorDistance > 5) {//rear sensor wall follow
    PIDControl (6,6);
    md.setM4Speed(100);
    RearRangeFinder();
  }
  BrakeMotor();   
}

