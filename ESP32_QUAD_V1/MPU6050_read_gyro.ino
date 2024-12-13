// Gyro config register
// To configure the full-scale range of the gyroscope
// ±500 deg/s = 65.5 LSB/s
// FS_SEL for ±500 = 1
// 8bit value = x=0 y=0 z=0 FS_SEL=1 Bit2=0 Bit1=0 Bit0=0
// Binary 0001000 to decimal = 8
#define GYRO_CONFIG_REG   0x1B
#define GYRO_CONFIG_VAL   0x08

// First of 6 gyro data registers 43 to 48.
// Sensitivity for ±500 deg/s = 65.5 LSB/s
#define GYRO_DATA_START   0x43
#define GYRO_SENSITIVITY  65.5

// Function to read gyroscope data and return it
void MPU6050_read_gyro() {
  // configure the full-scale range for the gyroscope measurements
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG_REG);
  Wire.write(GYRO_CONFIG_VAL);
  Wire.endTransmission();

  // start reading the data from first register 43
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_DATA_START);
  Wire.endTransmission();

  // Request 6 bytes of gyro data as there are 6 registers
  // 2 for each axis with 8bit and makes 16bit for each axis
  Wire.requestFrom(MPU_ADDR, 6);

  if (Wire.available() == 6) {
    int16_t x = Wire.read() << 8 | Wire.read(); // Read X-axis gyro data
    int16_t y = Wire.read() << 8 | Wire.read(); // Read Y-axis gyro data
    int16_t z = Wire.read() << 8 | Wire.read(); // Read Z-axis gyro data

    // Convert to degrees/sec
    rawGyroX = (float) x / GYRO_SENSITIVITY;
    rawGyroY = (float) y / GYRO_SENSITIVITY;
    rawGyroZ = (float) z / GYRO_SENSITIVITY;
  }
}

void MPU6050_print_gyro_data() {
  Serial.print("X: "); Serial.print(rawGyroX); Serial.print(" °/s ");
  Serial.print("Y: "); Serial.print(rawGyroY); Serial.print(" °/s ");
  Serial.print("Z: "); Serial.print(rawGyroZ); Serial.println(" °/s ");
}
