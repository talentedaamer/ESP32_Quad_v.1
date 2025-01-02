void MPU6050_calibrate_acce() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int32_t samples = 500;

  Serial.println(">> MPU6050 Calibrating Accelerometer");
  for (int i = 0; i < samples; i++) {
    MPU6050_read_acce();

    sumX += rawAcceX;
    sumY += rawAcceY;
    sumZ += rawAcceZ;

    delay(20);
  }

  // Calculate average
  acceOffsetX = sumX / samples;
  acceOffsetY = sumY / samples;
  acceOffsetZ = sumZ / samples;
  
  Serial.print("acceOffsetX: "); Serial.print(acceOffsetX); Serial.print(" g ");
  Serial.print("acceOffsetY: "); Serial.print(acceOffsetY); Serial.print(" g ");
  Serial.print("acceOffsetZ: "); Serial.print(acceOffsetZ); Serial.println(" g ");

  Serial.println(">> MPU6050 Accelerometer Calibrated");
  delay(250);
}
