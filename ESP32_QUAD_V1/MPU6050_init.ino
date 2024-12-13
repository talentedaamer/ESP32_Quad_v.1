#define MPU_PWR_MGMT_REG  0x6B // Power management register
#define MPU_PWR_MGMT_VAL  0x00 // set 0 to wake up MPU6050

// Initialize & Wake up the GY-87-MPU6050
// Wake up the MPU6050 since it starts in sleep mode
void MPU6050_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_REG);
  Wire.write(MPU_PWR_MGMT_VAL);
  Wire.endTransmission();
  Serial.println(">> MPU6050 Initialized");
  delay(250);
}