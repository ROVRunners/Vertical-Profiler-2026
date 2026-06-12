#include <Wire.h>
#include <MS5837.h>

const int I2C_SDA_PIN = 5;
const int I2C_SCL_PIN = 6;

MS5837 pressureSensor;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();

  Serial.println("Bar02 test starting...");

  if (!pressureSensor.init()) {
    Serial.println("Pressure sensor not found");
    while (1) delay(1000);
  }

  Serial.println("Pressure sensor found");

  pressureSensor.setModel(MS5837::MS5837_02BA);

  // Change this if needed
  pressureSensor.setFluidDensity(1023);
}

void loop() {
  pressureSensor.read();

  Serial.print("Pressure (mbar): ");
  Serial.println(pressureSensor.pressure(), 2);

  Serial.print("Temperature (C): ");
  Serial.println(pressureSensor.temperature(), 2);

  Serial.print("Depth (m): ");
  Serial.println(pressureSensor.depth(), 3);

  Serial.println();
  delay(1000);
}