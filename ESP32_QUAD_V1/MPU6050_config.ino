// MPU6050 config register for gyro and accel
// his register is used to configure the Digital Low-Pass Filter (DLPF)
// settings and set the external frame synchronization options for the sensor.
#define CONFIG_REG        0x1A // config register
#define CONFIG_REG_VAL    0x05 // 5 = Bandwidth 10Hz

// configure the Digital Low-Pass Filter (DLPF)
void MPU6050_config() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(CONFIG_REG_VAL);
  Wire.endTransmission();
  Serial.println(">> MPU6050 DLPF Configured");
  delay(250);
}