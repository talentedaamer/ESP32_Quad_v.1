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

// MPU6050 config register for gyro and accel
// his register is used to configure the Digital Low-Pass Filter (DLPF)
// settings and set the external frame synchronization options for the sensor.
#define CONFIG_REG        0x1A // config register
#define CONFIG_REG_VAL    0x05 // 5 = Bandwidth 10Hz

// Gyro config register
// To configure the full-scale range of the gyroscope
// ±500 deg/s = 65.5 LSB/s
// FS_SEL for ±500 = 1
// 8bit value = x=0 y=0 z=0 FS_SEL=1 Bit2=0 Bit1=0 Bit0=0
// Binary 0001000 to decimal = 8
#define GYRO_CONFIG_REG   0x1B
#define GYRO_CONFIG_VAL   0x08

// Accel config register
// To configure the full-scale range of the accelerometer
#define ACCEL_CONFIG_REG   0x1C
// ±8g = 4096 LSB/g
// FS_SEL for ±8g = 2
// Set FS_SEL 2 = 10
#define ACCEL_CONFIG_VAL   0x10

// configure the Digital Low-Pass Filter (DLPF)
void MPU6050_config() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(CONFIG_REG_VAL);
  Wire.endTransmission();
  Serial.println(">> MPU6050 DLPF Configured");
  delay(250);

  // configure the full-scale range for the gyroscope measurements
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG_REG);
  Wire.write(GYRO_CONFIG_VAL);
  Wire.endTransmission();
  Serial.println(">> MPU6050 Gyroscope full-scale range Configured");
  delay(250);

  // configure the full-scale range for the acceleration measurements
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG_REG);
  Wire.write(ACCEL_CONFIG_VAL);
  Wire.endTransmission();
  Serial.println(">> MPU6050 Acceleration full-scale range Configured");
  delay(250);
}
