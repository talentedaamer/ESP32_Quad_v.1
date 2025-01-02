#include <Wire.h>

// gyro and accelerometer x,y,z data
volatile float gyroX, gyroY, gyroZ;
volatile float acceX, acceY, acceZ;

// MPU6050 I2C address
#define MPU_ADDR        0x68
// Power management register
#define PWR_MGMT_1      0x6B
// set 0 to wake up MPU6050
#define PWR_MGMT_1_VAL  0x00

// Initialize & Wake up the GY-87-MPU6050
// Wake up the MPU6050 since it starts in sleep mode
void initializeMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(PWR_MGMT_1_VAL);
  Wire.endTransmission();
  Serial.println("========== MPU6050 Initialized ===========");
}

// MPU6050 config register for gyro and accel
// his register is used to configure the Digital Low-Pass Filter (DLPF)
// settings and set the external frame synchronization options for the sensor.
#define CONFIG_REG        0x1A
#define CONFIG_REG_VAL    0x05 // 5 = Bandwidth 10Hz
// Gyro config register
// To configure the full-scale range of the gyroscope
#define GYRO_CONFIG_REG   0x1B
// ±500 deg/s = 65.5 LSB/s
// FS_SEL for ±500 = 1
// 8bit value = x=0 y=0 z=0 FS_SEL=1 Bit2=0 Bit1=0 Bit0=0
// Binary 0001000 to decimal = 8
#define GYRO_CONFIG_VAL   0x08

// First of 6 gyro data registers 43 to 48.
#define GYRO_DATA_START   0x43

// Sensitivity for ±500 deg/s = 65.5 LSB/s
#define GYRO_SENSITIVITY  65.5

// Function to read gyroscope data and return it
void gy87_mpu6050_read_gyro_signals() {
  // configure the Digital Low-Pass Filter (DLPF)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(CONFIG_REG_VAL);
  Wire.endTransmission();

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
    gyroX = (float) x / GYRO_SENSITIVITY;
    gyroY = (float) y / GYRO_SENSITIVITY;
    gyroZ = (float) z / GYRO_SENSITIVITY;
  }

  // return {
  //   gyroX,
  //   gyroY,
  //   gyroZ,
  // };
}

// calibration bias to be removed from gyro valies
// for calibration on static position
float biasX = 0, biasY = 0, biasZ = 0;

// Function to calibrate gyroscope
void gy87_mpu6050_gyro_calibration(float* biasX, float* biasY, float* biasZ, int samples) {
  float sumX = 0, sumY = 0, sumZ = 0;
  
  Serial.println("==========================================");
  Serial.println("========= Calibrating gyroscope ==========");
  Serial.println("==========================================");

  for (int i = 0; i < samples; i++) {
      GyroData gyroData = gy87_mpu6050_read_gyro_signals();
      sumX += gyroData.roll;
      sumY += gyroData.pitch;
      sumZ += gyroData.yaw;
      delay(50);
  }

  *biasX = sumX / samples;
  *biasY = sumY / samples;
  *biasZ = sumZ / samples;

  Serial.print("Bias X: "); Serial.println(*biasX);
  Serial.print("Bias Y: "); Serial.println(*biasY);
  Serial.print("Bias Z: "); Serial.println(*biasZ);

  Serial.println("==========================================");
  Serial.println("========= Calibration complete ===========");
  Serial.println("==========================================");

}

// First of 6 temp data registers 41 & 42.
#define TEMP_DATA_START_ADDR  0x41
#define TEMP_SENSITIVITY      340.0
#define ROOM_TEMP             36.53

float gy87_mpu6050_read_temperature() {
  float tempCelsius = 0;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(TEMP_DATA_START_ADDR);
  Wire.endTransmission();

  Wire.requestFrom(MPU_ADDR, 2);

  if (Wire.available() == 2) {
    int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  
    // Convert raw temperature to Celsius
    tempCelsius = (rawTemp / TEMP_SENSITIVITY) + ROOM_TEMP;
    // tempCelsius = rawTemp;
  }

  return tempCelsius;
}

// Accel config register
// To configure the full-scale range of the accelerometer
#define ACCEL_CONFIG_REG   0x1C
// ±8g = 4096 LSB/g
// FS_SEL for ±8g = 2
// Set FS_SEL 2 = 10
#define ACCEL_CONFIG_VAL   0x10

// First of 6 accel data registers 3B, 3C, 3D, 3E, 3F, 40
#define ACCEL_DATA_START   0x3B

// Sensitivity for ±8g
#define ACCEL_SENSITIVITY  4096.0

AccelData gy87_mpu6050_read_accel_signals() {
  // create instance of AccelData
  AccelData accelData;
  
  int16_t rawX, rawY, rawZ;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG_REG);
  Wire.write(ACCEL_CONFIG_VAL);
  Wire.endTransmission();

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
    accelData.x = (float) rawX / ACCEL_SENSITIVITY;
    accelData.y = (float) rawY / ACCEL_SENSITIVITY;
    accelData.z = (float) rawZ / ACCEL_SENSITIVITY;
  }

  return accelData;
}

readBMP180

void setup() {
  // Start I2C
  Wire.begin();
  // Initialize serial communication
  Serial.begin(115200);
  // 400 kHz (Fast Mode):
  Wire.setClock(400000);
  // delay for quarter second
  delay(250);

  // Initialize MPU6050 and wait for quarter second
  initializeMPU6050();
  delay(250);

  // callibrate gyro
  // gy87_mpu6050_gyro_calibration(&biasX, &biasY, &biasZ, 100);
}
float roll, pitch;

void loop() {
  GyroData gyroData = gy87_mpu6050_read_gyro_signals();
  float temp = gy87_mpu6050_read_temperature();
  AccelData accelData = gy87_mpu6050_read_accel_signals();

  // calibration bias values
  // gyroData.roll -= biasX;
  // gyroData.pitch -= biasY;
  // gyroData.yaw -= biasZ;

  // Serial.print("Roll: "); Serial.print(gyroData.roll); Serial.print(" °/s ");
  // Serial.print("Pitch: "); Serial.print(gyroData.pitch); Serial.print(" °/s ");
  // Serial.print("Yaw: "); Serial.print(gyroData.yaw); Serial.println(" °/s ");

  // Serial.print("Temp: "); Serial.println(temp);

  // Serial.print("X: "); Serial.print(accelData.x); Serial.print(" °/s ");
  // Serial.print("Y: "); Serial.print(accelData.y); Serial.print(" °/s ");
  // Serial.print("Z: "); Serial.print(accelData.z); Serial.println(" °/s ");

  // AccelAngles angles;
  // angles.roll = atan( accelData.y / sqrt( accelData.x * accelData.x + accelData.z * accelData.z )) * 1 / (3.142/180);
  // angles.pitch = atan( accelData.x / sqrt( accelData.y * accelData.y + accelData.z * accelData.z )) * 1 / (3.142/180);
  
  // roll = atan( accelData.y / sqrt( accelData.x * accelData.x + accelData.z * accelData.z )) * 1 / (3.142/180);
  // pitch = atan( accelData.x / sqrt( accelData.y * accelData.y + accelData.z * accelData.z )) * 1 / (3.142/180);

  // Serial.print("Angle Roll [°]= "); Serial.print(roll);
  // Serial.print("Angle Pitch [°]= "); Serial.println(pitch);

  delay(50);
}

// 1. Initialization and Startup
//     Hardware Initialization:
//         Configure the AVR microcontroller (Atmel ATmega644PA on KK2.1.5).
//         Initialize GPIO pins for motor outputs, LED indicators, and button inputs.
//         Set up the I2C communication for sensor access.
//     Sensor Initialization:
//         Initialize onboard sensors (e.g., 6050 IMU with gyroscope and accelerometer).
//         Calibrate gyroscope and accelerometer for offset values.
//     EEPROM Load:
//         Load user settings (e.g., PIDs, mixer settings, throttle range) stored in the EEPROM.

// 2. Sensor Data Acquisition
//     IMU Data Reading:
//         Read raw gyroscope and accelerometer values from the MPU6050 IMU.
//     Sensor Calibration:
//         Apply calibration offsets to the raw sensor data.
//         Convert sensor data to angular rates (gyro) and linear acceleration (accelerometer).
//     Complementary Filter:
//         Fuse accelerometer and gyroscope data to calculate tilt angles (roll, pitch) and yaw.

// 3. User Input Handling
//     Receiver Input:
//         Read input signals (PWM or PPM) from the RC receiver.
//         Map channel signals to throttle, yaw, pitch, roll, and auxiliary functions.
//     Button and LCD Input:
//         Detect button presses for menu navigation or parameter adjustments.
//         Display menus and settings on the integrated LCD.

// 4. Flight Control Algorithm
//     PID Controller:
//         Apply the Proportional-Integral-Derivative (PID) algorithm to stabilize the craft:
//             Proportional (P): Correct errors in real-time.
//             Integral (I): Account for cumulative errors (drift).
//             Derivative (D): Smooth out control response by anticipating future errors.
//     Error Calculation:
//         Calculate the error between desired orientation (from user input) and current orientation (from sensors).
//     Mixer Output:
//         Map PID output values to motor speeds using the configured multirotor type (e.g., quadcopter, hexacopter).

// 5. Motor Control
//     ESC Signal Generation:
//         Generate PWM signals for ESCs (Electronic Speed Controllers).
//         Ensure appropriate update rates (e.g., 400 Hz for most ESCs).
//     Fail-Safe:
//         Cut throttle or stabilize the drone if the RC signal is lost or invalid.

// 6. Flight Modes
//     Stabilized Mode:
//         Uses the accelerometer to level the drone automatically.
//     Acro Mode:
//         Purely gyroscope-based; relies on the pilot for leveling.
//     Self-Leveling:
//         Combines accelerometer data to automatically correct tilt and keep the drone level.

// 7. Real-Time Processing
//     Sensor Loop:
//         Continuously read and process sensor data at high frequency.
//     Control Loop:
//         Run the PID controller and update motor outputs based on sensor data and user input.
//     LCD/Debug Loop:
//         Periodically update the LCD or output telemetry for debugging.

// 8. Firmware Update
//     Use a programmer like USBasp to flash the KK2.1.5 with new firmware (e.g., Steveis or other open-source options).
//     Steps for updating:
//         Download the firmware HEX file.
//         Connect the USBasp to the KK2.1.5 programming header.
//         Use a tool like kkMulticopter Flash Tool to upload the firmware.

// 9. Error Handling
//     Detect anomalies (e.g., sensor failure, low voltage, communication errors).
//     Trigger LED indicators or audible alarms.
//     Enter fail-safe mode if needed.

// 10. Tuning and Configuration
//     Use the onboard buttons and LCD to adjust settings:
//         PID values.
//         Throttle curve.
//         Motor mixing and orientation.
//         Calibration of sensors and ESCs.
//     Test flight and fine-tune based on drone behavior.