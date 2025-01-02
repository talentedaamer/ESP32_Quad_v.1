void ESP32_init() {
  // Start I2C
  Wire.begin();
  // Initialize serial communication
  Serial.begin(115200);
  // 400 kHz (Fast Mode)
  Wire.setClock(400000);
  delay(250);
}