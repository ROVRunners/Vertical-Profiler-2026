#include <ESP32Servo.h>

Servo buoyancyServo;

const int ACTUATOR_PIN = 8;

// PWM values
const int PRIME_HIGH_US = 2000;
const int PRIME_LOW_US = 1000;
const int NEUTRAL_TEST_US = 1500;

// Prime timing
const unsigned long PRIME_HIGH_TIME_MS = 1500;
const unsigned long PRIME_LOW_TIME_MS = 1500;
const unsigned long SETTLE_TIME_MS = 500;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Buoyancy tuning script starting...");

  buoyancyServo.setPeriodHertz(50);
  buoyancyServo.attach(ACTUATOR_PIN, 500, 2500);

  Serial.print("Priming high: ");
  Serial.println(PRIME_HIGH_US);
  buoyancyServo.writeMicroseconds(PRIME_HIGH_US);
  delay(PRIME_HIGH_TIME_MS);

  Serial.print("Priming low: ");
  Serial.println(PRIME_LOW_US);
  buoyancyServo.writeMicroseconds(PRIME_LOW_US);
  delay(PRIME_LOW_TIME_MS);

  delay(SETTLE_TIME_MS);

  Serial.print("Holding neutral test value: ");
  Serial.println(NEUTRAL_TEST_US);
  buoyancyServo.writeMicroseconds(NEUTRAL_TEST_US);

  Serial.println("Ready for neutral buoyancy tuning.");
}

void loop() {
  // Hold 1500 continuously
  buoyancyServo.writeMicroseconds(NEUTRAL_TEST_US);
  delay(200);
}