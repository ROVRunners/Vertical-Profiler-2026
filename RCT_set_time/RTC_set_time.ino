#include <Wire.h>
#include <RTClib.h>

const int I2C_SDA_PIN = 5;
const int I2C_SCL_PIN = 6;

RTC_DS1307 rtc;

void printDateTime(const DateTime& dt) {
  Serial.print(dt.year());
  Serial.print("-");
  if (dt.month() < 10) Serial.print("0");
  Serial.print(dt.month());
  Serial.print("-");
  if (dt.day() < 10) Serial.print("0");
  Serial.print(dt.day());
  Serial.print(" ");

  if (dt.hour() < 10) Serial.print("0");
  Serial.print(dt.hour());
  Serial.print(":");
  if (dt.minute() < 10) Serial.print("0");
  Serial.print(dt.minute());
  Serial.print(":");
  if (dt.second() < 10) Serial.print("0");
  Serial.println(dt.second());
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();

  Serial.println("RTC serial UTC setter starting...");

  if (!rtc.begin()) {
    Serial.println("RTC not found");
    while (1) delay(1000);
  }

  Serial.println("RTC found");
  Serial.println("Send Unix UTC time as seconds since 1970.");
  Serial.println("Example: 1775457600");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    unsigned long epoch = strtoul(input.c_str(), nullptr, 10);

    if (epoch > 1000000000UL) {
      rtc.adjust(DateTime(epoch));

      Serial.print("RTC set to UTC: ");
      printDateTime(rtc.now());
    } else {
      Serial.println("Invalid epoch received");
    }
  }
}