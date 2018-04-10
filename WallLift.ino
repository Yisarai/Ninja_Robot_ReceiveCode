void WallLift(){   
   RearRangeFinder();
   while (sensorDistance > 5) {//rear sensor wall follow
     PIDControl (2,2);
     md.setM4Speed(-100);
     RearRangeFinder();
   }
   BrakeMotor();   
}

