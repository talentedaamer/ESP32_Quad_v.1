// First of 6 gyro data registers 43 to 48.
// Sensitivity for ±500 deg/s = 65.5 LSB/s
#define GYRO_DATA_START   0x43
#define GYRO_SENSITIVITY  65.5

// Function to read gyroscope data and return it
void MPU6050_read_gyro() {
  int16_t rawX, rawY, rawZ;

  // start reading the data from first register 43
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_DATA_START);
  Wire.endTransmission();

  // Request 6 bytes of gyro data as there are 6 registers
  // 2 for each axis with 8bit and makes 16bit for each axis
  Wire.requestFrom(MPU_ADDR, 6);

  if (Wire.available() == 6) {
    rawX = Wire.read() << 8 | Wire.read(); // Read X-axis gyro data
    rawY = Wire.read() << 8 | Wire.read(); // Read Y-axis gyro data
    rawZ = Wire.read() << 8 | Wire.read(); // Read Z-axis gyro data

    // Convert to degrees/sec
    rawGyroX = (float) rawX / GYRO_SENSITIVITY;
    rawGyroY = (float) rawY / GYRO_SENSITIVITY;
    rawGyroZ = (float) rawZ / GYRO_SENSITIVITY;
  }
}
