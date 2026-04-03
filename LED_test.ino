#include <Adafruit_NeoPixel.h>

const int PIXEL_PIN = 10;
const int PIXEL_COUNT = 7;

Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setAll(uint32_t color) {
  for (int i = 0; i < PIXEL_COUNT; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pixels.begin();
  pixels.setBrightness(40);
  pixels.clear();
  pixels.show();

  Serial.println("NeoPixel test starting...");
}

void loop() {
  Serial.println("Red");
  setAll(pixels.Color(255, 0, 0));
  delay(1000);

  Serial.println("Green");
  setAll(pixels.Color(0, 255, 0));
  delay(1000);

  Serial.println("Blue");
  setAll(pixels.Color(0, 0, 255));
  delay(1000);

  Serial.println("White");
  setAll(pixels.Color(255, 255, 255));
  delay(1000);

  Serial.println("Off");
  setAll(pixels.Color(0, 0, 0));
  delay(1000);
}