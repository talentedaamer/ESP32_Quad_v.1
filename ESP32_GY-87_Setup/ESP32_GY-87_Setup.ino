#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

// Create objects for the sensors
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

// Standard Atmospheric Pressure at Sea Level: 101325 Pa (or 1013.25 hPa)
#define PRES_0M 101325.0

// Declare function prototypes
void doAcceleration(sensors_event_t accel);
void doGyro(sensors_event_t gyro);
void doPresAndTemp();
void setup();

int counter = 0;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Wait for a moment to stabilize
  delay(1000);

  // Initialize I2C communication
  Wire.begin();

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    while (1);
  }

  // Initialize BMP180 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1);
  }

  Serial.println("Sensors initialized successfully!");
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

void doPresAndTemp() {
  float temp, atm, alt;
  long pres;

  temp = bmp.readTemperature();
  pres = bmp.readPressure();

  atm = pres / PRES_0M; // "standard atmosphere"
  alt = bmp.readAltitude(PRES_0M);

  Serial.print("Temp: "); Serial.print(temp); Serial.println(" °C ");
  Serial.print("Pres: "); Serial.print(pres); Serial.print(" Pa "); Serial.print(" | ");
  Serial.print("Alt: "); Serial.print(alt); Serial.print(" m "); Serial.print(" | ");
  Serial.print("Rel Atm Pres: "); Serial.print(atm); Serial.println(" (rel atm)");
  // Serial.println("----------------------------------------------------------------");
}

void loop() {
  // increment counter
  counter++;

  // Get accelerometer and gyroscope data from MPU6050
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  
  doAcceleration(accel);
  doGyro(gyro);
  doPresAndTemp();

  Serial.println("============================== " + String(counter) + " ===============================");

  delay(5000);
}
