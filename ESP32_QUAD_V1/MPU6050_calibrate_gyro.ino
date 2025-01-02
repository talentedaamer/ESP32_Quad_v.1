void MPU6050_calibrate_gyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int32_t samples = 500;

  Serial.println(">> MPU6050 Calibrating Gyroscope...");
  for (int i = 0; i < samples; i++) {
    MPU6050_read_gyro();
    
    sumX += rawGyroX;
    sumY += rawGyroY;
    sumZ += rawGyroZ;

    delay(20);
  }

  // Calculate average
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  
  Serial.print("gyroOffsetX: "); Serial.print(gyroOffsetX); Serial.print(" °/s ");
  Serial.print("gyroOffsetY: "); Serial.print(gyroOffsetY); Serial.print(" °/s ");
  Serial.print("gyroOffsetZ: "); Serial.print(gyroOffsetZ); Serial.println(" °/s ");

  Serial.println(">> MPU6050 Gyroscope Calibrated");
  delay(250);
}
