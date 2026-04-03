#include <Wire.h>
#include <RTClib.h>

const int I2C_SDA_PIN = 5;
const int I2C_SCL_PIN = 6;

RTC_DS1307 rtc;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();

  Serial.println("RTC test starting...");

  if (!rtc.begin()) {
    Serial.println("RTC not found");
    while (1) delay(1000);
  }

  Serial.println("RTC found");

  if (!rtc.isrunning()) {
    Serial.println("RTC not running, setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  DateTime now = rtc.now();

  Serial.print(now.year());
  Serial.print("-");
  if (now.month() < 10) Serial.print("0");
  Serial.print(now.month());
  Serial.print("-");
  if (now.day() < 10) Serial.print("0");
  Serial.print(now.day());
  Serial.print(" ");

  if (now.hour() < 10) Serial.print("0");
  Serial.print(now.hour());
  Serial.print(":");
  if (now.minute() < 10) Serial.print("0");
  Serial.print(now.minute());
  Serial.print(":");
  if (now.second() < 10) Serial.print("0");
  Serial.println(now.second());

  delay(1000);
}