#include <ESP32Servo.h>

Servo buoyancyServo;

const int ACTUATOR_PIN = 8;

// Safe bounds
const int MIN_US = 500;
const int MAX_US = 2500;

// Startup / current value
int currentUs = 1500;

void setActuatorUs(int pulseUs) {
  if (pulseUs < MIN_US) pulseUs = MIN_US;
  if (pulseUs > MAX_US) pulseUs = MAX_US;

  currentUs = pulseUs;
  buoyancyServo.writeMicroseconds(currentUs);

  Serial.print("Actuator set to: ");
  Serial.print(currentUs);
  Serial.println(" us");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  buoyancyServo.setPeriodHertz(50);
  buoyancyServo.attach(ACTUATOR_PIN, 500, 2500);

  // Start at neutral
  setActuatorUs(currentUs);

  Serial.println("Buoyancy engine serial control ready.");
  Serial.println("Send a number from 500 to 2500.");
  Serial.println("Example: 1500");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    int value = input.toInt();

    if (value >= MIN_US && value <= MAX_US) {
      setActuatorUs(value);
    } else {
      Serial.println("Invalid value. Enter 500 to 2500.");
    }
  }
}