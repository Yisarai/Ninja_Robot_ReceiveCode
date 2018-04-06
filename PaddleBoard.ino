Encoder myEnc1(20,21);  
Encoder myEnc2(18,19);  //create instances for each encoder
float GearRatio = 131;            //the gear ratio
int countsPerRev_motor = 64;      //the counts per revolution of the motor shaft
int counts1 = 0;                   //Globally intialize encoder counts
int counts2 = 0;                   //Globally intialize encoder counts
int f = 2;      
