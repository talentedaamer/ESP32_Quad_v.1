// First of 6 accel data registers 3B, 3C, 3D, 3E, 3F, 40
// Sensitivity for ±8g
#define ACCEL_DATA_START   0x3B
#define ACCEL_SENSITIVITY  4096.0

// Function to read gyroscope data and return it
void MPU6050_read_acce() {
  int16_t rawX, rawY, rawZ;

  // start reading the data from first register 3B
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_DATA_START);
  Wire.endTransmission();

  // Request 6 bytes of accel data as there are 6 registers
  // 2 for each axis with 8bit and makes 16bit for each axis
  Wire.requestFrom(MPU_ADDR, 6);

  if (Wire.available() == 6) {
    rawX = Wire.read() << 8 | Wire.read(); // Read X-axis accel data
    rawY = Wire.read() << 8 | Wire.read(); // Read Y-axis accel data
    rawZ = Wire.read() << 8 | Wire.read(); // Read Z-axis accel data

    // Convert to degrees/sec
    rawAcceX = (float) rawX / ACCEL_SENSITIVITY;
    rawAcceY = (float) rawY / ACCEL_SENSITIVITY;
    rawAcceZ = (float) rawZ / ACCEL_SENSITIVITY;
  }
}
