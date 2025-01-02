#include <Wire.h>

// Calibration coefficients
int16_t AC1, AC2, AC3, BMP_B1, BMP_B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;


// Functions to read/write registers
uint8_t read8(uint8_t reg) {
    Wire.beginTransmission(0x77);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(0x77, 1);
    return Wire.read();
}

uint16_t read16(uint8_t reg) {
    Wire.beginTransmission(0x77);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(0x77, 2);
    uint16_t value = (Wire.read() << 8) | Wire.read();
    return value;
}

uint32_t read24(uint8_t reg) {
    Wire.beginTransmission(0x77);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(0x77, 3);
    uint32_t value = ((uint32_t)Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
    return value;
}

void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(0x77);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Read calibration data
void readCalibrationData() {
    AC1 = read16(0xAA);
    AC2 = read16(0xAC);
    AC3 = read16(0xAE);
    AC4 = read16(0xB0);
    AC5 = read16(0xB2);
    AC6 = read16(0xB4);
    BMP_B1 = read16(0xB6);
    BMP_B2 = read16(0xB8);
    MB  = read16(0xBA);
    MC  = read16(0xBC);
    MD  = read16(0xBE);
}

// Calculate temperature
float calculateTemperature(uint16_t rawTemp) {
    int32_t X1 = ((rawTemp - AC6) * AC5) >> 15;
    int32_t X2 = (MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;
    return ((B5 + 8) >> 4) / 10.0;  // Temperature in °C
}

// Calculate pressure
float calculatePressure(uint32_t rawPressure, uint16_t rawTemp, uint8_t OSS) {
    int32_t X1, X2, X3, B3, B5, B6, P;
    uint32_t B4, B7;

    // Calculate B5 using rawTemp
    X1 = ((rawTemp - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;

    B6 = B5 - 4000;
    X1 = (BMP_B2 * (B6 * B6 >> 12)) >> 11; 
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((AC1 * 4 + X3) << OSS) + 2) >> 2;

    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;

    B7 = ((uint32_t)rawPressure - B3) * (50000 >> OSS);
    if (B7 < 0x80000000) {
        P = (B7 * 2) / B4;
    } else {
        P = (B7 / B4) * 2;
    }
    X1 = (P >> 8) * (P >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * P) >> 16;
    P += (X1 + X2 + 3791) >> 4;

    return P / 100.0;  // Pressure in hPa
}

// Read BMP180 data
void readBMP180() {
    // Read temperature
    writeRegister(0xF4, 0x2E);  // Start temperature conversion
    delay(5);                   // Wait for conversion
    uint16_t rawTemp = read16(0xF6);
    float temperature = calculateTemperature(rawTemp);

    // Read pressure
    uint8_t OSS = 3;            // Oversampling setting (0–3)
    writeRegister(0xF4, 0x34 + (OSS << 6));
    delay(26);                  // Wait for conversion (max time for OSS=3)
    uint32_t rawPressure = read24(0xF6) >> (8 - OSS);
    float pressure = calculatePressure(rawPressure, rawTemp, OSS);

    // Print results
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
}

void setup() {
    Wire.begin();
    Serial.begin(115200);
    readCalibrationData();  // Load BMP180 calibration data
}

void loop() {
    readBMP180();  // Read and print BMP180 data
    delay(1000);   // Delay 1 second
}
