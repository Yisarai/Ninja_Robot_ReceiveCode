void WallLift(){   
   RearRangeFinder();
   while (sensorDistance > 5) {//rear sensor wall follow
     RearRangeFinder();
     md.setM1Speed(350);
     md.setM2Speed(-350);
     md.setM4Speed(-100);
   }
}

