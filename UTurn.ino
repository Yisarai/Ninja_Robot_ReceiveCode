void LineFollow() {
  qtrrc.read(sensorValues);
  float num = 0;
  float den = 0;

  for (unsigned char i = 0; i < NUM_SENSORS; i++) {

    sensorValueBiased[i] = sensorValues[i] - sensorBias[i];
    num = num + sensorValueBiased[i] * i;
    den = den + sensorValueBiased[i];

    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  float lineLoc = num / den - 4.5;
  Serial.println(lineLoc);
  if (lineLoc < - 0.5) {
    Serial.println("Turn left");
    //Left Motor
    md.setM1Speed(100);
    //Right Motor
    md.setM2Speed(0);
  }
  else if (lineLoc > 0.5) {
    Serial.println("Turn Right");

       //Right Motor
    md.setM2Speed(-100);
    //Left Motor
    md.setM1Speed(0);
  }
  else {
    Serial.println("Drive Straight");
    //Left Motor
    md.setM1Speed(45);
    //Right Motor
    md.setM2Speed(-45);
  }

  delay(10);
}

