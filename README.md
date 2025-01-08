# ESP32_Quad v.1

## Description

**ESP32_Quad v.1** is an open-source project developed using the **ESP32 WROOM32 Dev Kit V1** to interface with the **GY-87** which contains **(MPU6050, HMC5883L, and BMP180)** sensor (accelerometer, gyroscope, and magnetometer). The project is designed for drone or quadcopter applications, where sensor data is read directly from the **MPU9250** using its register map, without relying on any external libraries. The project utilizes the **Arduino IDE** for development, providing a straightforward way to handle sensor initialization, calibration, and data collection for further processing in quadcopter control algorithms.

### Features:
- **Sensor Initialization**: Initializes the **MPU6050** via I2C communication.
- **Accelerometer and Gyroscope Calibration**: Includes calibration routines for both accelerometer and gyroscope to ensure data accuracy.
- **Data Reading**: Directly reads accelerometer and gyroscope data from the **MPU6050** sensor registers.
- **I2C Communication**: Data is fetched from the sensor over I2C without relying on any additional sensor libraries.

This project is ideal for hobbyists or developers looking to build their own flight control system for drones or other robotic systems using the **ESP32** and the **MPU6050** sensor.

## Project Structure

The project contains the following files:

- **ESP32_QUAD_V1.ino**: Main file containing the setup and control loop for the ESP32.
- **ESP32_init.ino**: Initializes the ESP32 and sets up I2C communication.
- **MPU6050_init.ino**: Initializes the **MPU6050** sensor for data acquisition. Set gyroscope and accelerometer configurations
- **MPU6050_read_gyro.ino**: Reads gyroscope data from the **MPU6050**.
- **MPU6050_calibrate_gyro.ino**: Calibrates the gyroscope in the **MPU6050** sensor.
- **MPU6050_read_acce.ino**: Reads accelerometer data from the **MPU6050**.
- **MPU6050_calibrate_acce.ino**: Calibrates the accelerometer in the **MPU6050** sensor.

## Hardware Requirements

- **ESP32 WROOM32 Dev Kit V1**
- **MPU6050 Sensor** (or **MPU6050** if using only accelerometer and gyroscope data)
- **Wires** to connect the ESP32 to the **MPU6050** over **I2C**

## Software Requirements

- **Arduino IDE**
- **ESP32 Board Support** (to be added via the Arduino IDE Board Manager)

## Installation

1. **Set up the Arduino IDE**:
   - Install the **ESP32 Board** support in the Arduino IDE by going to **File** → **Preferences**, and adding the following URL to the **Additional Boards Manager URLs**:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Go to **Tools** → **Board** → **Boards Manager**, search for **ESP32** and install the **ESP32** board package.

2. **Clone the repository** to your local machine:
   ```bash
   git clone https://github.com/talentedaamer/ESP32_Quad_v.1.git

3. **Open the project** in the Arduino IDE.

4. **Connect the hardware**:
   - Connect the **MPU6050** sensor to the **ESP32** using the **I2C** bus (SCL and SDA pins).

5. **Upload the code** to your **ESP32** board.

## Usage

- The **MPU6050** sensor is initialized via the **MPU6050_init.ino** file. This sets up the sensor for I2C communication and prepares it to start transmitting data.
- Calibration routines for both the accelerometer and gyroscope are handled in the **MPU6050_calibrate_acce.ino** and **MPU6050_calibrate_gyro.ino** files, respectively.
- Sensor data can be read from the **MPU6050** using the **MPU6050_read_acce.ino** and **MPU6050_read_gyro.ino** files, providing raw sensor data for further processing.

You can expand the project by adding more functionality such as **sensor fusion algorithms**, **PID control** for flight stabilization, or integrating additional sensors like **GPS** for navigation.

## Next Steps

1. **Sensor Fusion**: Implement sensor fusion algorithms like **Complementary Filter**, or **Kalman filter**  to combine accelerometer, gyroscope, and magnetometer data.
2. **Flight Control**: Integrate the sensor data into a **PID control loop** or another flight control algorithm to stabilize a drone or quadcopter.
3. **Additional Sensors**: Add more sensors such as a **barometer** for altitude or a **GPS module** for navigation and positioning.

## Contributing

Feel free to fork the repository and submit pull requests with improvements, new features, or bug fixes. If you encounter any issues or have suggestions, please open an issue.

## License

This project is open-source and licensed under the **MIT License**.

