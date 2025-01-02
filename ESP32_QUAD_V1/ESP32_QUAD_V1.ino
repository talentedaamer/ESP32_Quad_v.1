// Microcontroller (MCU): ESP32 WROOM32
// Inertial Measurement Unit (IMU): GY-87 10DOF (MPU6050, HMC5883L, BMP180)
// Functionality:
// 1. Capture raw gyroscope data.
// 2. Calibrate the gyroscope to eliminate offsets.
// 3. Capture raw accelerometer data.
// 4. Calibrate the accelerometer to remove bias.
// Perform sensor fusion using a Kalman filter for accurate angle estimation.

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

#include <Wire.h>

// Gyroscope reading variables
volatile float rawGyroX, rawGyroY, rawGyroZ;
volatile float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
volatile float gyroX, gyroY, gyroZ;
// Acceleration reading variables
volatile float rawAcceX, rawAcceY, rawAcceZ;
volatile float acceOffsetX, acceOffsetY, acceOffsetZ;
volatile float acceX, acceY, acceZ;

// I2C Communication
#define MPU_ADDR  0x68 // MPU6050 I2C address

// Initialize ESP32 & MPU6050
void ESP32_init();
void MPU6050_init();
// read and calibrate gyro
void MPU6050_read_gyro();
void MPU6050_calibrate_gyro();
// read and calibrate acce
void MPU6050_read_acce();
void MPU6050_calibrate_acce();

void setup() {
  // ESP32 Initialize
  ESP32_init();
  // MPU6050 Initialize & config DLPF
  MPU6050_init();
  // MPU6050 calibrate
  // gyroscope & accelerometer
  MPU6050_calibrate_gyro();
  MPU6050_calibrate_acce();
}

void loop() {
  // MPU6050 read gyro signals
  MPU6050_read_gyro();
  // MPU6050_print_gyro_data();
  gyroX = rawGyroX - gyroOffsetX;
  gyroY = rawGyroY - gyroOffsetY;
  gyroZ = rawGyroZ - gyroOffsetZ;
  // print final gyro data
  // Serial.print("X: "); Serial.print(gyroX);
  // Serial.print(" Y: "); Serial.print(gyroY);
  // Serial.print(" Z: "); Serial.println(gyroZ);

  // MPU6050 read acce signals
  MPU6050_read_acce();
  acceX = rawAcceX - acceOffsetX;
  acceY = rawAcceY - acceOffsetY;
  acceZ = rawAcceZ - acceOffsetZ;
  // print final gyro data
  // Serial.print("X: "); Serial.print(acceX);
  // Serial.print(" Y: "); Serial.print(acceY);
  // Serial.print(" Z: "); Serial.println(acceZ);

  delay(50);
}


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
