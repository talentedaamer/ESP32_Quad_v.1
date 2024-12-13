float calculate_average(float* array, int length);
float prevGyroX, prevGyroY, prevGyroZ;
float xGyroBuffer[1000], yGyroBuffer[1000], zGyroBuffer[1000];

void MPU6050_calibrate_gyro() {
  for (int i = 0; i < 1000; i++) {
    // read gyro data each time
    MPU6050_read_gyro();

    if (
      rawGyroX != prevGyroX ||
      rawGyroY != prevGyroY ||
      rawGyroZ != prevGyroZ
    ) {
      // Store raw values in a buffer
      xGyroBuffer[i] = rawGyroX;
      yGyroBuffer[i] = rawGyroY;
      zGyroBuffer[i] = rawGyroZ;

      // Serial.print("X: "); Serial.print(rawGyroX); Serial.print(" °/s ");
      // Serial.print("Y: "); Serial.print(rawGyroY); Serial.print(" °/s ");
      // Serial.print("Z: "); Serial.print(rawGyroZ); Serial.println(" °/s ");
      
      // Update previous values for the next iteration
      prevGyroX = rawGyroX;
      prevGyroY = rawGyroY;
      prevGyroZ = rawGyroZ;
    }

    delay(10);
  }

  // Calculate the average of the raw values
  gyroOffsetX = calculate_average(xGyroBuffer, 1000);
  gyroOffsetY = calculate_average(yGyroBuffer, 1000);
  gyroOffsetZ = calculate_average(zGyroBuffer, 1000);
  
  Serial.print("gyroOffsetX: "); Serial.print(gyroOffsetX); Serial.print(" °/s ");
  Serial.print("gyroOffsetY: "); Serial.print(gyroOffsetY); Serial.print(" °/s ");
  Serial.print("gyroOffsetZ: "); Serial.print(gyroOffsetZ); Serial.println(" °/s ");

  Serial.println(">> MPU6050 Gyroscope Calibrated");
  delay(250);
}

float calculate_average(float* array, int length) {
  float sum = 0;
  for (int i = 0; i < length; i++) {
    sum += array[i];
  }
  return sum / length;
}