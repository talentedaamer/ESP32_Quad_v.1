#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>

// Create objects for the sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified();
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Standard Atmospheric Pressure at Sea Level: 101325 Pa (or 1013.25 hPa)
#define PRES_0M 101325.0

// Declare function prototypes
void displayBmpSensorDetails(void);
void displayMagSensorDetails(void);

void doAcceleration(sensors_event_t accel);
void doMagnetometer(sensors_event_t event);
void doGyro(sensors_event_t gyro);
void doPresAndTempAlt();
void setup();

/*************************************************************************
                        BMP Sensor Information
*************************************************************************/
void displayBmpSensorDetails(void) {
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("--------------------------------------------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("--------------------------------------------------------------------------");
  Serial.println("");
  delay(500);
}

/*************************************************************************
                        Mag Sensor Information
*************************************************************************/
void displayMagSensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("--------------------------------------------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("--------------------------------------------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Wait for a second to stabilize
  delay(1000);

  // Initialize I2C communication
  Wire.begin();

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU6050 sensor");
    while (1);
  }

  // Initialize BMP085 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor");
    while (1);
  }

  // Initialize HMC5883 sensor
  if(!mag.begin()) {
    Serial.println("Could not find a valid HMC5883 sensor");
    while(1);
  }
  
  // Enable I2C bypass mode
  mpu.setI2CBypassEnabled(true);

  displayBmpSensorDetails();
  displayMagSensorDetails();
}

void doAcceleration(sensors_event_t accel) {
  if (accel.acceleration.x != 0 || accel.acceleration.y != 0 || accel.acceleration.z != 0) {
    float accX = accel.acceleration.x;
    float accY = accel.acceleration.y;
    float accZ = accel.acceleration.z;

    Serial.print("Acc");
    Serial.print(" X: "); Serial.print(accX); Serial.print(" m/s^2 "); Serial.print(" | ");
    Serial.print(" Y: "); Serial.print(accY); Serial.print(" m/s^2 "); Serial.print(" | ");
    Serial.print(" Z: "); Serial.print(accZ); Serial.println(" m/s^2 ");
    // Serial.println("----------------------------------------------------------------");
  } else {
    Serial.println("No acceleration data");
  }
}

void doGyro(sensors_event_t gyro) {
  if (gyro.gyro.x != 0 || gyro.gyro.y != 0 || gyro.gyro.z != 0) {
    float gyroX = gyro.gyro.x;
    float gyroY = gyro.gyro.y;
    float gyroZ = gyro.gyro.z;
    
    Serial.print("Gyro");
    Serial.print(" X: "); Serial.print(gyroX); Serial.print(" dps "); Serial.print(" | ");
    Serial.print(" Y: "); Serial.print(gyroY); Serial.print(" dps "); Serial.print(" | ");
    Serial.print(" Z: "); Serial.print(gyroZ); Serial.println(" dps ");
    // Serial.println("----------------------------------------------------------------");
  } else {
    Serial.println("No gyro data");
  }
}

void doPresAndTempAlt() {
  float temperature, pressure;
  bmp.getTemperature(&temperature);
  bmp.getPressure(&pressure);

  float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
  float altitude = bmp.pressureToAltitude(seaLevelPressure, pressure);

  Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa "); Serial.print(" | ");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" °C "); Serial.print(" | ");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m ");
  // Serial.println("--------------------------------------------------------------------------");
  Serial.println("==========================================================================");
}

void doMagnetometer(sensors_event_t event) {
  float magX = event.magnetic.x;
  float magY = event.magnetic.y;
  float magZ = event.magnetic.z;

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(magY, magX);

  // Add your 'Declination Angle'
  float declinationAngle = 0.22; // Adjust this value based on your location
  heading += declinationAngle;

  // Correct for when signs are reversed
  if (heading < 0)
      heading += 2 * PI;

  // Check for wrap due to addition of declination
  if (heading > 2 * PI)
      heading -= 2 * PI;

  // Convert radians to degrees for readability
  float headingDegrees = heading * 180 / M_PI; 

  Serial.print("Mag: ");
  Serial.print(" X: "); Serial.print(magX); Serial.print(" ut "); Serial.print(" | ");
  Serial.print(" Y: "); Serial.print(magY); Serial.print(" ut "); Serial.print(" | ");
  Serial.print(" Z: "); Serial.print(magZ); Serial.println(" ut ");
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  // Serial.println("--------------------------------------------------------------------------");
}

void loop() {
  sensors_event_t accel, gyro, temp, magEvent;
  mpu.getEvent(&accel, &gyro, &temp);
  mag.getEvent(&magEvent);
  
  doAcceleration(accel);
  doMagnetometer(magEvent);
  doGyro(gyro);
  doPresAndTempAlt();
  

  delay(1000);
}
