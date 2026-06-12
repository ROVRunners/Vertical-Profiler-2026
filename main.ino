#include <WiFi.h>
#include <Wire.h>
#include <RTClib.h>
#include <MS5837.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ===== Team / mission info =====
const char* TEAM_ID = "EX01";

// ===== Mission options =====
const bool SELF_RECOVER_TO_SURFACE = true;   // false = stay at 0.40 m, true = surface after final hold

// ===== WiFi AP config =====
const char* ssid = "VP_Float";

WiFiServer server(80);

// Static IP
IPAddress local_IP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// ===== I2C pins =====
const int I2C_SDA_PIN = 5;
const int I2C_SCL_PIN = 6;

// ===== RTC =====
RTC_DS1307 rtc;

// ===== Pressure sensor =====
MS5837 pressureSensor;

// ===== Buoyancy actuator =====
Servo buoyancyServo;
const int ACTUATOR_PIN = 8;

// Actuator tuning
const int ACTUATOR_MIN_US = 500;
const int ACTUATOR_MAX_US = 2500;
const int ACTUATOR_NEUTRAL_US = 1500;
const int ACTUATOR_IDLE_US = 500;

// Startup priming
const int PRIME_HIGH_US = 2000;
const int PRIME_LOW_US = 1000;
const unsigned long PRIME_HIGH_TIME_MS = 1500;
const unsigned long PRIME_LOW_TIME_MS = 1500;
const unsigned long PRIME_SETTLE_TIME_MS = 500;

// Manual control
bool manualModeEnabled = false;
int manualCommandUs = ACTUATOR_NEUTRAL_US;

// If larger PWM makes the float go deeper, leave this as +1.
// If it goes the wrong way, change to -1.
const int CONTROL_DIRECTION = 1;

int actuatorCommandUs = -1;

// ===== PID tuning =====
float PID_KP = 400.0f;
float PID_KI = 10.0f;
float PID_KD = 1100.0f;

// Integral clamp to prevent windup
float PID_INTEGRAL_MIN = -1.5f;
float PID_INTEGRAL_MAX =  1.5f;

// PID update rate
const unsigned long PID_INTERVAL_MS = 100;

// ===== Mission targets =====
float TARGET_DEEP_M = 2.50f;
float TARGET_SHALLOW_M = 0.40f;
float TARGET_SURFACE_M = 0.02f;
unsigned long HOLD_TIME_MS = 32000;
float TARGET_TOLERANCE_M = 0.15f;
float SURFACE_TOLERANCE_M = 0.05f;
unsigned long MAX_TRANSIT_TIME_MS = 90000;
unsigned long MAX_HOLD_TIME_MS = 60000;
unsigned long MAX_RECOVER_TIME_MS = 90000;

// ===== Zeroing =====
const int ZERO_SAMPLE_COUNT = 20;
const unsigned long ZERO_SAMPLE_DELAY_MS = 50;
float depthZeroOffset_m = 0.0f;

// ===== NeoPixel Jewel =====
const int PIXEL_PIN = 10;
const int PIXEL_COUNT = 7;
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

float liveVelocity_mps = 0.0f;

// ===== LED behavior =====
const float MAX_DISPLAY_VELOCITY_MPS = 0.20f;
const unsigned long LED_BLINK_INTERVAL_MS = 400;
bool zeroingInProgress = false;
const unsigned long ZEROING_BLINK_INTERVAL_MS = 50;

// ===== Mission state machine =====
enum State {
  IDLE,
  DESCEND_1,
  HOLD_250_1,
  ASCEND_1,
  HOLD_040_1,
  DESCEND_2,
  HOLD_250_2,
  ASCEND_2,
  HOLD_040_2,
  STATION_KEEP_040,
  RECOVER_SURFACE,
  MANUAL
};

State currentState = IDLE;

// ===== Logging =====
const int MAX_SAMPLES = 2000;
unsigned long timeLog[MAX_SAMPLES];
float depthLog[MAX_SAMPLES];
int sampleCount = 0;

bool loggingEnabled = false;
unsigned long loggingStartMillis = 0;
unsigned long lastSampleMillis = 0;
const unsigned long SAMPLE_INTERVAL_MS = 200;

// ===== Hold timing =====
unsigned long stateEntryMillis = 0;
unsigned long inToleranceStartMillis = 0;
bool inToleranceTimerRunning = false;

// ===== PID state =====
float pidIntegral = 0.0f;
float pidPrevError = 0.0f;
unsigned long lastPidMillis = 0;

// ===== Live values =====
float livePressure_mbar = 0.0f;
float liveTemp_C = 0.0f;
float rawDepth_m = 0.0f;
float liveDepth_m = 0.0f;

// ===== RTC display =====
DateTime nowRTC;

// ===== Function declarations =====
void handleClient();
void runStateMachine();
void updateSensors();
void startMission();
void stopLoggingAndIdle();
void setActuatorUs(int pulseUs);
void updateStateLEDs();
void enterState(State newState);
const char* stateName(State s);
void primeBuoyancyEngine();
bool stateTimedOut(unsigned long maxTimeMs);

float clampFloat(float x, float lo, float hi);
uint32_t blendColor(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, float t);
bool stateHasTarget(State s);
float getCurrentTargetDepth();
float getCurrentTargetTolerance();

void sendManual(WiFiClient& client);
void handleSetManualMode(WiFiClient& client, const String& request);
void handleSetManualUs(WiFiClient& client, const String& request);
int getQueryParamInt(const String& request, const String& key, int currentValue);
void sendMission(WiFiClient& client);
void handleSetMission(WiFiClient& client, const String& request);
unsigned long getQueryParamULong(const String& request, const String& key, unsigned long currentValue);

void resetPID();
void runDepthPID(float targetDepthM);
void sendPID(WiFiClient& client);
void handleSetPID(WiFiClient& client, const String& request);
float getQueryParam(const String& request, const String& key, float currentValue);
void zeroDepthSensor();
void logSampleIfNeeded();

bool isWithinTolerance(float targetDepthM, float toleranceM);
bool holdCompleteAtTarget(float targetDepthM, float toleranceM, unsigned long holdTimeMs);

void sendHeader(WiFiClient& client, const char* contentType);
void sendOK(WiFiClient& client, const char* msg);
void sendElapsedTime(WiFiClient& client);
void sendRTC(WiFiClient& client);
void sendPressure(WiFiClient& client);
void sendStatus(WiFiClient& client);
void sendData(WiFiClient& client);
void sendPage(WiFiClient& client);

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();

  Serial.println("Starting setup...");

  // ----- NeoPixel Jewel -----
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(40);
  updateStateLEDs();
  Serial.println("NeoPixel ready.");

  // ----- RTC -----
  if (!rtc.begin()) {
    Serial.println("RTC not found. Check wiring.");
    while (1) delay(1000);
  }
  Serial.println("RTC found.");

  if (!rtc.isrunning()) {
    Serial.println("RTC lost power, setting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // ----- Pressure sensor -----
  if (!pressureSensor.init()) {
    Serial.println("Pressure sensor not found. Check wiring.");
    while (1) delay(1000);
  }
  Serial.println("Pressure sensor found.");

  pressureSensor.setModel(MS5837::MS5837_02BA);

  // Water density, change if needed
  pressureSensor.setFluidDensity(1023);

  // First read
  pressureSensor.read();
  livePressure_mbar = pressureSensor.pressure();
  liveTemp_C = pressureSensor.temperature();
  rawDepth_m = pressureSensor.depth();
  liveDepth_m = rawDepth_m;

  // ----- Buoyancy actuator -----
  buoyancyServo.setPeriodHertz(50);
  buoyancyServo.attach(ACTUATOR_PIN, 500, 2500);
  setActuatorUs(ACTUATOR_IDLE_US);
  Serial.println("Servo attached.");

  // Prime / clear buoyancy engine at startup
  primeBuoyancyEngine();

  // ----- WiFi AP -----
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid);

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

// ===== Main loop =====
void loop() {
  handleClient();
  updateSensors();
  runStateMachine();
  logSampleIfNeeded();
  updateStateLEDs();
}

// ===== State helpers =====
void enterState(State newState) {
  currentState = newState;
  stateEntryMillis = millis();
  inToleranceStartMillis = 0;
  inToleranceTimerRunning = false;
  resetPID();
  updateStateLEDs();

  Serial.print("Entered state: ");
  Serial.println(stateName(currentState));
}

const char* stateName(State s) {
  switch (s) {
    case IDLE:             return "IDLE";
    case DESCEND_1:        return "DESCEND_1";
    case HOLD_250_1:       return "HOLD_250_1";
    case ASCEND_1:         return "ASCEND_1";
    case HOLD_040_1:       return "HOLD_040_1";
    case DESCEND_2:        return "DESCEND_2";
    case HOLD_250_2:       return "HOLD_250_2";
    case ASCEND_2:         return "ASCEND_2";
    case HOLD_040_2:       return "HOLD_040_2";
    case STATION_KEEP_040: return "STATION_KEEP_040";
    case RECOVER_SURFACE:  return "RECOVER_SURFACE";
    case MANUAL:           return "MANUAL";
    default:               return "UNKNOWN";
  }
}

// ===== Actuator helper =====
void setActuatorUs(int pulseUs) {
  if (pulseUs < ACTUATOR_MIN_US) pulseUs = ACTUATOR_MIN_US;
  if (pulseUs > ACTUATOR_MAX_US) pulseUs = ACTUATOR_MAX_US;

  if (pulseUs != actuatorCommandUs) {
    actuatorCommandUs = pulseUs;
    buoyancyServo.writeMicroseconds(actuatorCommandUs);

    Serial.print("Actuator command: ");
    Serial.print(actuatorCommandUs);
    Serial.println(" us");
  }
}

// ===== Startup prime =====
void primeBuoyancyEngine() {
  Serial.println("Priming buoyancy engine...");

  setActuatorUs(PRIME_HIGH_US);
  delay(PRIME_HIGH_TIME_MS);

  setActuatorUs(PRIME_LOW_US);
  delay(PRIME_LOW_TIME_MS);

  setActuatorUs(ACTUATOR_IDLE_US);
  delay(PRIME_SETTLE_TIME_MS);

  Serial.println("Prime complete.");
}

// ===== NeoPixel state helper =====
void updateStateLEDs() {
  static unsigned long lastBlinkToggle = 0;
  static bool blinkOn = true;

  unsigned long now = millis();

  unsigned long blinkInterval = LED_BLINK_INTERVAL_MS;
  if (zeroingInProgress) {
    blinkInterval = ZEROING_BLINK_INTERVAL_MS;
  }

  if (now - lastBlinkToggle >= blinkInterval) {
    lastBlinkToggle = now;
    blinkOn = !blinkOn;
  }

  pixels.clear();

  // ----- ZEROING OVERRIDE -----
  if (zeroingInProgress) {
    uint32_t c = blinkOn ? pixels.Color(255, 255, 0) : pixels.Color(0, 0, 0);
    for (int i = 0; i < PIXEL_COUNT; i++) {
      pixels.setPixelColor(i, c);
    }
    pixels.show();
    return;
  }

  // ----- MANUAL MODE OVERRIDE -----
  if (currentState == MANUAL || manualModeEnabled) {
    uint32_t c = blinkOn ? pixels.Color(255, 255, 0) : pixels.Color(0, 0, 0);
    for (int i = 0; i < PIXEL_COUNT; i++) {
      pixels.setPixelColor(i, c);
    }
    pixels.show();
    return;
  }

  // ----- VELOCITY: LEDs 1-2 (pixels 0,1) -----
  // Upward = yellow, downward = purple
  // liveVelocity_mps > 0 means going deeper
  float velNorm = clampFloat(liveVelocity_mps / MAX_DISPLAY_VELOCITY_MPS, -1.0f, 1.0f);
  float velT = (velNorm + 1.0f) * 0.5f;

  uint32_t velColor = blendColor(
    255, 255,   0,   // yellow = full up
    160,   0, 255,   // purple = full down
    velT
  );

  pixels.setPixelColor(0, velColor);
  pixels.setPixelColor(1, velColor);

  // ----- STATE LED ONLY: LED 3 (pixel 2) -----
  uint32_t stateColor = pixels.Color(0, 0, 0);

  switch (currentState) {
    case IDLE:
      stateColor = blinkOn ? pixels.Color(0, 0, 255) : pixels.Color(0, 0, 0);   // one blinking blue idle LED
      break;

    case DESCEND_1:
    case DESCEND_2:
      stateColor = pixels.Color(255, 0, 255);   // magenta
      break;

    case HOLD_250_1:
    case HOLD_250_2:
      stateColor = pixels.Color(255, 0, 0);     // red
      break;

    case ASCEND_1:
    case ASCEND_2:
      stateColor = pixels.Color(255, 180, 0);   // amber
      break;

    case HOLD_040_1:
    case HOLD_040_2:
      stateColor = pixels.Color(0, 255, 0);     // green
      break;

    case STATION_KEEP_040:
      stateColor = pixels.Color(255, 255, 255); // white
      break;

    case RECOVER_SURFACE:
      stateColor = pixels.Color(0, 255, 255);   // cyan
      break;

    default:
      stateColor = pixels.Color(0, 0, 0);
      break;
  }

  pixels.setPixelColor(2, stateColor);

  // ----- POSITION RELATIVE TO TARGET BOUNDS: LEDs 4-5 (pixels 3,4) -----
  uint32_t posColor = pixels.Color(0, 0, 0);

  if (stateHasTarget(currentState)) {
    float target = getCurrentTargetDepth();
    float tol = getCurrentTargetTolerance();
    float errorAbs = fabs(liveDepth_m - target);

    if (errorAbs > tol) {
      posColor = blinkOn ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0);   // blinking red outside bounds
    } else {
      float closeness = 1.0f - (errorAbs / tol);  // 1 at target, 0 at tolerance edge
      posColor = blendColor(
        0,   0,   0,     // black
        255, 40, 180,    // pink
        closeness
      );
    }
  }

  pixels.setPixelColor(3, posColor);
  pixels.setPixelColor(4, posColor);

  // ----- ACTUATOR COMMAND: LEDs 6-7 (pixels 5,6) -----
  float actT = (float)(actuatorCommandUs - ACTUATOR_MIN_US) /
               (float)(ACTUATOR_MAX_US - ACTUATOR_MIN_US);
  actT = clampFloat(actT, 0.0f, 1.0f);

  uint32_t actuatorColor = blendColor(
      0,  80, 255,   // blue
    255, 120,   0,   // orange
    actT
  );

  pixels.setPixelColor(5, actuatorColor);
  pixels.setPixelColor(6, actuatorColor);

  pixels.show();
}

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

uint32_t blendColor(uint8_t r1, uint8_t g1, uint8_t b1,
                    uint8_t r2, uint8_t g2, uint8_t b2,
                    float t) {
  t = clampFloat(t, 0.0f, 1.0f);

  uint8_t r = (uint8_t)(r1 + (r2 - r1) * t);
  uint8_t g = (uint8_t)(g1 + (g2 - g1) * t);
  uint8_t b = (uint8_t)(b1 + (b2 - b1) * t);

  return pixels.Color(r, g, b);
}

bool stateHasTarget(State s) {
  switch (s) {
    case DESCEND_1:
    case HOLD_250_1:
    case DESCEND_2:
    case HOLD_250_2:
    case ASCEND_1:
    case HOLD_040_1:
    case ASCEND_2:
    case HOLD_040_2:
    case STATION_KEEP_040:
    case RECOVER_SURFACE:
      return true;
    default:
      return false;
  }
}

float getCurrentTargetDepth() {
  switch (currentState) {
    case DESCEND_1:
    case HOLD_250_1:
    case DESCEND_2:
    case HOLD_250_2:
      return TARGET_DEEP_M;

    case ASCEND_1:
    case HOLD_040_1:
    case ASCEND_2:
    case HOLD_040_2:
    case STATION_KEEP_040:
      return TARGET_SHALLOW_M;

    case RECOVER_SURFACE:
      return TARGET_SURFACE_M;

    default:
      return 0.0f;
  }
}

float getCurrentTargetTolerance() {
  if (currentState == RECOVER_SURFACE) {
    return SURFACE_TOLERANCE_M;
  }
  return TARGET_TOLERANCE_M;
}

// ===== Sensor update =====
void updateSensors() {
  static unsigned long lastUpdate = 0;
  static float prevDepth_m = 0.0f;
  static bool firstRun = true;

  unsigned long now = millis();
  if (now - lastUpdate >= 100) {
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    pressureSensor.read();
    livePressure_mbar = pressureSensor.pressure();
    liveTemp_C = pressureSensor.temperature();
    rawDepth_m = pressureSensor.depth();
    liveDepth_m = rawDepth_m - depthZeroOffset_m;

    if (firstRun || dt <= 0.0f) {
      liveVelocity_mps = 0.0f;
      firstRun = false;
    } else {
      liveVelocity_mps = (liveDepth_m - prevDepth_m) / dt;
    }

    prevDepth_m = liveDepth_m;
  }
}

// ===== PID =====
void resetPID() {
  pidIntegral = 0.0f;
  pidPrevError = 0.0f;
  lastPidMillis = millis();
}

void runDepthPID(float targetDepthM) {
  unsigned long now = millis();
  if (now - lastPidMillis < PID_INTERVAL_MS) return;

  float dt = (now - lastPidMillis) / 1000.0f;
  lastPidMillis = now;

  float error = targetDepthM - liveDepth_m;

  pidIntegral += error * dt;
  if (pidIntegral > PID_INTEGRAL_MAX) pidIntegral = PID_INTEGRAL_MAX;
  if (pidIntegral < PID_INTEGRAL_MIN) pidIntegral = PID_INTEGRAL_MIN;

  float derivative = 0.0f;
  if (dt > 0.0f) {
    derivative = (error - pidPrevError) / dt;
  }
  pidPrevError = error;

  float outputUs =
      CONTROL_DIRECTION *
      (PID_KP * error + PID_KI * pidIntegral + PID_KD * derivative);

  int commandUs = (int)(ACTUATOR_NEUTRAL_US + outputUs);
  setActuatorUs(commandUs);
}

// ===== Zero depth =====
void zeroDepthSensor() {
  float sum = 0.0f;

  Serial.println("Zeroing depth sensor...");
  zeroingInProgress = true;

  for (int i = 0; i < ZERO_SAMPLE_COUNT; i++) {
    pressureSensor.read();
    sum += pressureSensor.depth();

    updateStateLEDs();   // show fast blinking during zeroing
    delay(ZERO_SAMPLE_DELAY_MS);
  }

  depthZeroOffset_m = sum / ZERO_SAMPLE_COUNT;

  Serial.print("Depth zero offset (m): ");
  Serial.println(depthZeroOffset_m, 4);

  pressureSensor.read();
  rawDepth_m = pressureSensor.depth();
  liveDepth_m = rawDepth_m - depthZeroOffset_m;

  zeroingInProgress = false;
  updateStateLEDs();
}

// ===== Logging =====
void logSampleIfNeeded() {
  if (!loggingEnabled) return;

  if (millis() - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    lastSampleMillis = millis();

    if (sampleCount < MAX_SAMPLES) {
      timeLog[sampleCount] = millis() - loggingStartMillis;
      depthLog[sampleCount] = liveDepth_m;
      sampleCount++;
    }
  }
}

// ===== Hold helpers =====
bool isWithinTolerance(float targetDepthM, float toleranceM) {
  return fabs(liveDepth_m - targetDepthM) <= toleranceM;
}

bool holdCompleteAtTarget(float targetDepthM, float toleranceM, unsigned long holdTimeMs) {
  if (isWithinTolerance(targetDepthM, toleranceM)) {
    if (!inToleranceTimerRunning) {
      inToleranceTimerRunning = true;
      inToleranceStartMillis = millis();
    }

    if (millis() - inToleranceStartMillis >= holdTimeMs) {
      return true;
    }
  } else {
    inToleranceTimerRunning = false;
    inToleranceStartMillis = 0;
  }

  return false;
}

// ===== Mission control =====
void startMission() {
  manualModeEnabled = false;

  sampleCount = 0;
  loggingEnabled = true;
  loggingStartMillis = millis();
  lastSampleMillis = 0;

  zeroDepthSensor();
  enterState(DESCEND_1);

  Serial.println("Mission started");
}

void stopLoggingAndIdle() {
  manualModeEnabled = false;
  loggingEnabled = false;
  enterState(IDLE);
  setActuatorUs(ACTUATOR_IDLE_US);

  Serial.println("Logging stopped / returned to IDLE");
  Serial.print("Samples logged: ");
  Serial.println(sampleCount);
}

bool stateTimedOut(unsigned long maxTimeMs) {
  return (millis() - stateEntryMillis) >= maxTimeMs;
}

// ===== State Machine =====
void runStateMachine() {
  switch (currentState) {
    case IDLE:
      setActuatorUs(ACTUATOR_IDLE_US);
      break;

    case DESCEND_1:
      runDepthPID(TARGET_DEEP_M);

      if (isWithinTolerance(TARGET_DEEP_M, TARGET_TOLERANCE_M)) {
        enterState(HOLD_250_1);
      }
      else if (stateTimedOut(MAX_TRANSIT_TIME_MS)) {
        Serial.println("DESCEND_1 timeout -> HOLD_250_1");
        enterState(HOLD_250_1);
      }
      break;

    case HOLD_250_1:
      runDepthPID(TARGET_DEEP_M);

      if (holdCompleteAtTarget(TARGET_DEEP_M, TARGET_TOLERANCE_M, HOLD_TIME_MS)) {
        enterState(ASCEND_1);
      }
      else if (stateTimedOut(MAX_HOLD_TIME_MS)) {
        Serial.println("HOLD_250_1 timeout -> ASCEND_1");
        enterState(ASCEND_1);
      }
      break;

    case ASCEND_1:
      runDepthPID(TARGET_SHALLOW_M);

      if (isWithinTolerance(TARGET_SHALLOW_M, TARGET_TOLERANCE_M)) {
        enterState(HOLD_040_1);
      }
      else if (stateTimedOut(MAX_TRANSIT_TIME_MS)) {
        Serial.println("ASCEND_1 timeout -> HOLD_040_1");
        enterState(HOLD_040_1);
      }
      break;

    case HOLD_040_1:
      runDepthPID(TARGET_SHALLOW_M);

      if (holdCompleteAtTarget(TARGET_SHALLOW_M, TARGET_TOLERANCE_M, HOLD_TIME_MS)) {
        enterState(DESCEND_2);
      }
      else if (stateTimedOut(MAX_HOLD_TIME_MS)) {
        Serial.println("HOLD_040_1 timeout -> DESCEND_2");
        enterState(DESCEND_2);
      }
      break;

    case DESCEND_2:
      runDepthPID(TARGET_DEEP_M);

      if (isWithinTolerance(TARGET_DEEP_M, TARGET_TOLERANCE_M)) {
        enterState(HOLD_250_2);
      }
      else if (stateTimedOut(MAX_TRANSIT_TIME_MS)) {
        Serial.println("DESCEND_2 timeout -> HOLD_250_2");
        enterState(HOLD_250_2);
      }
      break;

    case HOLD_250_2:
      runDepthPID(TARGET_DEEP_M);

      if (holdCompleteAtTarget(TARGET_DEEP_M, TARGET_TOLERANCE_M, HOLD_TIME_MS)) {
        enterState(ASCEND_2);
      }
      else if (stateTimedOut(MAX_HOLD_TIME_MS)) {
        Serial.println("HOLD_250_2 timeout -> ASCEND_2");
        enterState(ASCEND_2);
      }
      break;

    case ASCEND_2:
      runDepthPID(TARGET_SHALLOW_M);

      if (isWithinTolerance(TARGET_SHALLOW_M, TARGET_TOLERANCE_M)) {
        enterState(HOLD_040_2);
      }
      else if (stateTimedOut(MAX_TRANSIT_TIME_MS)) {
        Serial.println("ASCEND_2 timeout -> HOLD_040_2");
        enterState(HOLD_040_2);
      }
      break;

    case HOLD_040_2:
      runDepthPID(TARGET_SHALLOW_M);

      if (holdCompleteAtTarget(TARGET_SHALLOW_M, TARGET_TOLERANCE_M, HOLD_TIME_MS)) {
        if (SELF_RECOVER_TO_SURFACE) {
          enterState(RECOVER_SURFACE);
        } else {
          enterState(STATION_KEEP_040);
        }
      }
      else if (stateTimedOut(MAX_HOLD_TIME_MS)) {
        Serial.println("HOLD_040_2 timeout -> next state");
        if (SELF_RECOVER_TO_SURFACE) {
          enterState(RECOVER_SURFACE);
        } else {
          enterState(STATION_KEEP_040);
        }
      }
      break;

    case STATION_KEEP_040:
      runDepthPID(TARGET_SHALLOW_M);
      break;

    case RECOVER_SURFACE:
      runDepthPID(TARGET_SURFACE_M);

      if (isWithinTolerance(TARGET_SURFACE_M, SURFACE_TOLERANCE_M)) {
        setActuatorUs(ACTUATOR_IDLE_US);
      }
      else if (stateTimedOut(MAX_RECOVER_TIME_MS)) {
        Serial.println("RECOVER_SURFACE timeout -> IDLE");
        setActuatorUs(ACTUATOR_IDLE_US);
        enterState(IDLE);
      }
      break;

    case MANUAL:
      setActuatorUs(manualCommandUs);
      break;
  }
}

// ===== Handle Web Requests =====
void handleClient() {
  WiFiClient client = server.available();
  if (!client) return;

  String request = client.readStringUntil('\r');
  client.readStringUntil('\n');

  Serial.println(request);

  if (request.indexOf("GET /start") >= 0) {
    if (!loggingEnabled && currentState == IDLE) {
      startMission();
    }
    sendOK(client, "STARTED");
  }
  else if (request.indexOf("GET /stop") >= 0) {
    stopLoggingAndIdle();
    sendOK(client, "STOPPED");
  }
  else if (request.indexOf("GET /time") >= 0) {
    sendElapsedTime(client);
  }
  else if (request.indexOf("GET /rtc") >= 0) {
    sendRTC(client);
  }
  else if (request.indexOf("GET /pressure") >= 0) {
    sendPressure(client);
  }
  else if (request.indexOf("GET /status") >= 0) {
    sendStatus(client);
  }
  else if (request.indexOf("GET /data") >= 0) {
    sendData(client);
  }
  else if (request.indexOf("GET /pid") >= 0) {
    sendPID(client);
  }
  else if (request.indexOf("GET /setpid") >= 0) {
    handleSetPID(client, request);
  }
    else if (request.indexOf("GET /manual") >= 0) {
    sendManual(client);
  }
  else if (request.indexOf("GET /setmanualmode") >= 0) {
    handleSetManualMode(client, request);
  }
  else if (request.indexOf("GET /setmanualus") >= 0) {
    handleSetManualUs(client, request);
  }
  else if (request.indexOf("GET /mission") >= 0) {
    sendMission(client);
  }
  else if (request.indexOf("GET /setmission") >= 0) {
    handleSetMission(client, request);
  }
  else {
    sendPage(client);
  }

  client.stop();
}

// ===== HTTP Responses =====
void sendHeader(WiFiClient& client, const char* contentType) {
  client.println("HTTP/1.1 200 OK");
  client.print("Content-type:");
  client.println(contentType);
  client.println("Connection: close");
  client.println();
}

void sendOK(WiFiClient& client, const char* msg) {
  sendHeader(client, "text/plain");
  client.println(msg);
}

void sendElapsedTime(WiFiClient& client) {
  sendHeader(client, "text/plain");

  if (loggingEnabled) {
    client.println((millis() - loggingStartMillis) / 1000.0, 2);
  } else if (sampleCount > 0) {
    client.println(timeLog[sampleCount - 1] / 1000.0, 2);
  } else {
    client.println("0.00");
  }
}

void sendRTC(WiFiClient& client) {
  nowRTC = rtc.now();

  char buffer[25];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
           nowRTC.year(), nowRTC.month(), nowRTC.day(),
           nowRTC.hour(), nowRTC.minute(), nowRTC.second());

  sendHeader(client, "text/plain");
  client.println(buffer);
}

void sendPressure(WiFiClient& client) {
  sendHeader(client, "text/plain");
  client.println(livePressure_mbar, 2);
}

void sendStatus(WiFiClient& client) {
  sendHeader(client, "application/json");

  client.print("{");
  client.print("\"state\":\"");
  client.print(stateName(currentState));
  client.print("\",");

  client.print("\"team_id\":\"");
  client.print(TEAM_ID);
  client.print("\",");

  client.print("\"samples\":");
  client.print(sampleCount);
  client.print(",");

  client.print("\"logging\":");
  client.print(loggingEnabled ? "true" : "false");
  client.print(",");

  client.print("\"self_recover\":");
  client.print(SELF_RECOVER_TO_SURFACE ? "true" : "false");
  client.print(",");

  client.print("\"pressure_mbar\":");
  client.print(livePressure_mbar, 2);
  client.print(",");

  client.print("\"depth_m\":");
  client.print(liveDepth_m, 3);
  client.print(",");

  client.print("\"temp_c\":");
  client.print(liveTemp_C, 2);
  client.print(",");

  client.print("\"actuator_us\":");
  client.print(actuatorCommandUs);

  client.print(",");
  client.print("\"manual_mode\":");
  client.print(manualModeEnabled ? "true" : "false");
  client.print(",");

  client.print("\"manual_us\":");
  client.print(manualCommandUs);

  client.print("}");
}

void sendData(WiFiClient& client) {
  sendHeader(client, "application/json");

  client.print("{\"time\":[");
  for (int i = 0; i < sampleCount; i++) {
    client.print(timeLog[i]);
    if (i < sampleCount - 1) client.print(",");
  }

  client.print("],\"depth\":[");
  for (int i = 0; i < sampleCount; i++) {
    client.print(depthLog[i], 3);
    if (i < sampleCount - 1) client.print(",");
  }
  client.print("]}");
}

float getQueryParam(const String& request, const String& key, float currentValue) {
  String token = key + "=";
  int start = request.indexOf(token);
  if (start < 0) return currentValue;

  start += token.length();
  int end = request.indexOf('&', start);
  if (end < 0) end = request.indexOf(' ', start);
  if (end < 0) end = request.length();

  String valueStr = request.substring(start, end);
  valueStr.trim();

  if (valueStr.length() == 0) return currentValue;
  return valueStr.toFloat();
}

int getQueryParamInt(const String& request, const String& key, int currentValue) {
  String token = key + "=";
  int start = request.indexOf(token);
  if (start < 0) return currentValue;

  start += token.length();
  int end = request.indexOf('&', start);
  if (end < 0) end = request.indexOf(' ', start);
  if (end < 0) end = request.length();

  String valueStr = request.substring(start, end);
  valueStr.trim();

  if (valueStr.length() == 0) return currentValue;
  return valueStr.toInt();
}

unsigned long getQueryParamULong(const String& request, const String& key, unsigned long currentValue) {
  String token = key + "=";
  int start = request.indexOf(token);
  if (start < 0) return currentValue;

  start += token.length();
  int end = request.indexOf('&', start);
  if (end < 0) end = request.indexOf(' ', start);
  if (end < 0) end = request.length();

  String valueStr = request.substring(start, end);
  valueStr.trim();

  if (valueStr.length() == 0) return currentValue;
  return (unsigned long)valueStr.toInt();
}

void sendPID(WiFiClient& client) {
  sendHeader(client, "application/json");

  client.print("{");
  client.print("\"kp\":");
  client.print(PID_KP, 4);
  client.print(",");
  client.print("\"ki\":");
  client.print(PID_KI, 4);
  client.print(",");
  client.print("\"kd\":");
  client.print(PID_KD, 4);
  client.print("}");
}

void handleSetPID(WiFiClient& client, const String& request) {
  PID_KP = getQueryParam(request, "kp", PID_KP);
  PID_KI = getQueryParam(request, "ki", PID_KI);
  PID_KD = getQueryParam(request, "kd", PID_KD);

  resetPID();

  Serial.print("Updated PID -> Kp: ");
  Serial.print(PID_KP, 4);
  Serial.print(" Ki: ");
  Serial.print(PID_KI, 4);
  Serial.print(" Kd: ");
  Serial.println(PID_KD, 4);

  sendPID(client);
}

void sendMission(WiFiClient& client) {
  sendHeader(client, "application/json");

  client.print("{");
  client.print("\"deep_m\":");
  client.print(TARGET_DEEP_M, 3);
  client.print(",");

  client.print("\"shallow_m\":");
  client.print(TARGET_SHALLOW_M, 3);
  client.print(",");

  client.print("\"surface_m\":");
  client.print(TARGET_SURFACE_M, 3);
  client.print(",");

  client.print("\"hold_ms\":");
  client.print(HOLD_TIME_MS);
  client.print(",");

  client.print("\"tol_m\":");
  client.print(TARGET_TOLERANCE_M, 3);
  client.print(",");

  client.print("\"surface_tol_m\":");
  client.print(SURFACE_TOLERANCE_M, 3);
  client.print(",");

  client.print("\"max_transit_ms\":");
  client.print(MAX_TRANSIT_TIME_MS);
  client.print(",");

  client.print("\"max_hold_ms\":");
  client.print(MAX_HOLD_TIME_MS);
  client.print(",");

  client.print("\"max_recover_ms\":");
  client.print(MAX_RECOVER_TIME_MS);

  client.print("}");
}

void handleSetMission(WiFiClient& client, const String& request) {
  TARGET_DEEP_M        = getQueryParam(request, "deep", TARGET_DEEP_M);
  TARGET_SHALLOW_M     = getQueryParam(request, "shallow", TARGET_SHALLOW_M);
  TARGET_SURFACE_M     = getQueryParam(request, "surface", TARGET_SURFACE_M);
  HOLD_TIME_MS         = getQueryParamULong(request, "hold", HOLD_TIME_MS);
  TARGET_TOLERANCE_M   = getQueryParam(request, "tol", TARGET_TOLERANCE_M);
  SURFACE_TOLERANCE_M  = getQueryParam(request, "stol", SURFACE_TOLERANCE_M);
  MAX_TRANSIT_TIME_MS  = getQueryParamULong(request, "maxtransit", MAX_TRANSIT_TIME_MS);
  MAX_HOLD_TIME_MS     = getQueryParamULong(request, "maxhold", MAX_HOLD_TIME_MS);
  MAX_RECOVER_TIME_MS  = getQueryParamULong(request, "maxrecover", MAX_RECOVER_TIME_MS);

  // Basic sanity guards
  if (TARGET_DEEP_M < 0.0f) TARGET_DEEP_M = 0.0f;
  if (TARGET_SHALLOW_M < 0.0f) TARGET_SHALLOW_M = 0.0f;
  if (TARGET_SURFACE_M < 0.0f) TARGET_SURFACE_M = 0.0f;

  if (TARGET_TOLERANCE_M < 0.001f) TARGET_TOLERANCE_M = 0.001f;
  if (SURFACE_TOLERANCE_M < 0.001f) SURFACE_TOLERANCE_M = 0.001f;

  if (HOLD_TIME_MS < 1000) HOLD_TIME_MS = 1000;
  if (MAX_TRANSIT_TIME_MS < 1000) MAX_TRANSIT_TIME_MS = 1000;
  if (MAX_HOLD_TIME_MS < 1000) MAX_HOLD_TIME_MS = 1000;
  if (MAX_RECOVER_TIME_MS < 1000) MAX_RECOVER_TIME_MS = 1000;

  // Reset timing/PID so current state uses new settings cleanly
  stateEntryMillis = millis();
  inToleranceStartMillis = 0;
  inToleranceTimerRunning = false;
  resetPID();

  Serial.println("Mission settings updated:");
  Serial.print("Deep: "); Serial.println(TARGET_DEEP_M, 3);
  Serial.print("Shallow: "); Serial.println(TARGET_SHALLOW_M, 3);
  Serial.print("Surface: "); Serial.println(TARGET_SURFACE_M, 3);
  Serial.print("Hold ms: "); Serial.println(HOLD_TIME_MS);
  Serial.print("Tol m: "); Serial.println(TARGET_TOLERANCE_M, 3);
  Serial.print("Surface tol m: "); Serial.println(SURFACE_TOLERANCE_M, 3);
  Serial.print("Max transit ms: "); Serial.println(MAX_TRANSIT_TIME_MS);
  Serial.print("Max hold ms: "); Serial.println(MAX_HOLD_TIME_MS);
  Serial.print("Max recover ms: "); Serial.println(MAX_RECOVER_TIME_MS);

  sendMission(client);
}

void sendManual(WiFiClient& client) {
  sendHeader(client, "application/json");

  client.print("{");
  client.print("\"enabled\":");
  client.print(manualModeEnabled ? "true" : "false");
  client.print(",");
  client.print("\"manual_us\":");
  client.print(manualCommandUs);
  client.print("}");
}

void handleSetManualMode(WiFiClient& client, const String& request) {
  int enabled = getQueryParamInt(request, "enabled", manualModeEnabled ? 1 : 0);

  if (enabled == 1) {
    manualModeEnabled = true;
    loggingEnabled = false;
    enterState(MANUAL);
    setActuatorUs(manualCommandUs);
  } else {
    manualModeEnabled = false;
    enterState(IDLE);
    setActuatorUs(ACTUATOR_IDLE_US);
  }

  sendManual(client);
}

void handleSetManualUs(WiFiClient& client, const String& request) {
  manualCommandUs = getQueryParamInt(request, "us", manualCommandUs);

  if (manualCommandUs < ACTUATOR_MIN_US) manualCommandUs = ACTUATOR_MIN_US;
  if (manualCommandUs > ACTUATOR_MAX_US) manualCommandUs = ACTUATOR_MAX_US;

  if (manualModeEnabled && currentState == MANUAL) {
    setActuatorUs(manualCommandUs);
  }

  sendManual(client);
}

// ===== Web page =====
void sendPage(WiFiClient& client) {
  sendHeader(client, "text/html");

  client.println(R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Vertical Profiler</title>
<style>
body {
  font-family: Helvetica, Arial, sans-serif;
  text-align: center;
  margin: 20px;
}
.button {
  color: white;
  padding: 16px 28px;
  font-size: 22px;
  border: none;
  cursor: pointer;
  margin: 8px;
}
.startBtn { background-color: #2e8b57; }
.stopBtn  { background-color: #b22222; }
.card {
  margin: 12px auto;
  max-width: 760px;
  padding: 14px;
  border: 1px solid #ccc;
  border-radius: 10px;
}
.big {
  font-size: 32px;
  margin: 10px 0;
}
.med {
  font-size: 22px;
  margin: 10px 0;
}
canvas {
  border: 1px solid #aaa;
  max-width: 100%;
  background: #fff;
}
</style>
</head>
<body>

<h1>Vertical Profiler</h1>

<div class="card">
  <div class="med" id="team">Team: EX01</div>
  <button class="button startBtn" onclick="startLog()">START LOGGING</button>
  <button class="button stopBtn" onclick="stopLog()">STOP LOGGING</button>
</div>

<div class="card">
  <div class="big" id="elapsed">0.00 s</div>
  <div class="med" id="rtc">UTC: --</div>
  <div class="med" id="pressure">Pressure: -- mbar</div>
  <div class="med" id="depth">Depth: -- m</div>
  <div class="med" id="state">State: --</div>
  <div class="med" id="samples">Samples: 0</div>
  <div class="med" id="actuator">Actuator: -- us</div>
</div>

<div class="card">
  <h2>Depth vs Time</h2>
  <canvas id="graph" width="700" height="350"></canvas>
</div>

<div class="card">
  <h2>Mission Settings</h2>
  <div class="med">Deep target (m): <input id="deepTarget" type="number" step="0.01"></div>
  <div class="med">Shallow target (m): <input id="shallowTarget" type="number" step="0.01"></div>
  <div class="med">Surface target (m): <input id="surfaceTarget" type="number" step="0.01"></div>
  <div class="med">Hold time (ms): <input id="holdMs" type="number" step="1000"></div>
  <div class="med">Target tolerance (m): <input id="tolM" type="number" step="0.01"></div>
  <div class="med">Surface tolerance (m): <input id="surfaceTolM" type="number" step="0.01"></div>
  <div class="med">Max transit (ms): <input id="maxTransitMs" type="number" step="1000"></div>
  <div class="med">Max hold (ms): <input id="maxHoldMs" type="number" step="1000"></div>
  <div class="med">Max recover (ms): <input id="maxRecoverMs" type="number" step="1000"></div>
  <button class="button startBtn" onclick="applyMission()">APPLY MISSION</button>
  <div class="med" id="missionStatus">Mission: --</div>
</div>

<div class="card">
  <h2>PID Tuning</h2>
  <div class="med">Kp: <input id="kp" type="number" step="0.1"></div>
  <div class="med">Ki: <input id="ki" type="number" step="0.1"></div>
  <div class="med">Kd: <input id="kd" type="number" step="0.1"></div>
  <button class="button startBtn" onclick="applyPID()">APPLY PID</button>
  <div class="med" id="pidStatus">PID: --</div>
</div>

<div class="card">
  <h2>Manual Control</h2>
  <div class="med">Mode: <span id="manualMode">OFF</span></div>
  <div class="med">
    Command (us):
    <input id="manualUs" type="number" step="1" min="500" max="2500" value="1500">
  </div>
  <button class="button startBtn" onclick="setManualMode(true)">MANUAL ON</button>
  <button class="button stopBtn" onclick="setManualMode(false)">MANUAL OFF</button>
  <button class="button startBtn" onclick="applyManualUs()">SEND COMMAND</button>
</div>

<script>
function startLog() {
  fetch('/start').then(() => {
    clearGraph();
  });
}

function stopLog() {
  fetch('/stop')
    .then(() => fetch('/data'))
    .then(res => res.json())
    .then(data => drawGraph(data.time, data.depth));
}

function updateElapsed() {
  fetch('/time')
    .then(res => res.text())
    .then(data => {
      document.getElementById('elapsed').innerText = data + ' s';
    });
}

function updateRTC() {
  fetch('/rtc')
    .then(res => res.text())
    .then(data => {
      document.getElementById('rtc').innerText = 'RTC: ' + data;
    });
}

function updatePressure() {
  fetch('/pressure')
    .then(res => res.text())
    .then(data => {
      document.getElementById('pressure').innerText = 'Pressure: ' + data + ' mbar';
    });
}

function updateStatus() {
  fetch('/status')
    .then(res => res.json())
    .then(data => {
      document.getElementById('team').innerText = 'Team: ' + data.team_id;
      document.getElementById('state').innerText = 'State: ' + data.state;
      document.getElementById('samples').innerText = 'Samples: ' + data.samples;
      document.getElementById('actuator').innerText = 'Actuator: ' + data.actuator_us + ' us';
      document.getElementById('depth').innerText = 'Depth: ' + data.depth_m.toFixed(3) + ' m';
      document.getElementById('manualMode').innerText = data.manual_mode ? 'ON' : 'OFF';
    });
}

function clearGraph() {
  const canvas = document.getElementById('graph');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#000';
  ctx.font = '18px Helvetica';
  ctx.fillText('No data yet', 20, 30);
}

function loadManual() {
  fetch('/manual')
    .then(res => res.json())
    .then(data => {
      document.getElementById('manualMode').innerText = data.enabled ? 'ON' : 'OFF';
      document.getElementById('manualUs').value = data.manual_us;
    });
}

function setManualMode(enabled) {
  fetch(`/setmanualmode?enabled=${enabled ? 1 : 0}`)
    .then(res => res.json())
    .then(data => {
      document.getElementById('manualMode').innerText = data.enabled ? 'ON' : 'OFF';
      document.getElementById('manualUs').value = data.manual_us;
    });
}

function applyManualUs() {
  const us = document.getElementById('manualUs').value;
  fetch(`/setmanualus?us=${encodeURIComponent(us)}`)
    .then(res => res.json())
    .then(data => {
      document.getElementById('manualMode').innerText = data.enabled ? 'ON' : 'OFF';
      document.getElementById('manualUs').value = data.manual_us;
    });
}

function loadMission() {
  fetch('/mission')
    .then(res => res.json())
    .then(data => {
      document.getElementById('deepTarget').value = data.deep_m;
      document.getElementById('shallowTarget').value = data.shallow_m;
      document.getElementById('surfaceTarget').value = data.surface_m;
      document.getElementById('holdMs').value = data.hold_ms;
      document.getElementById('tolM').value = data.tol_m;
      document.getElementById('surfaceTolM').value = data.surface_tol_m;
      document.getElementById('maxTransitMs').value = data.max_transit_ms;
      document.getElementById('maxHoldMs').value = data.max_hold_ms;
      document.getElementById('maxRecoverMs').value = data.max_recover_ms;

      document.getElementById('missionStatus').innerText =
        `Mission: deep=${data.deep_m} m, shallow=${data.shallow_m} m, hold=${data.hold_ms} ms`;
    });
}

function applyMission() {
  const deep = document.getElementById('deepTarget').value;
  const shallow = document.getElementById('shallowTarget').value;
  const surface = document.getElementById('surfaceTarget').value;
  const hold = document.getElementById('holdMs').value;
  const tol = document.getElementById('tolM').value;
  const stol = document.getElementById('surfaceTolM').value;
  const maxtransit = document.getElementById('maxTransitMs').value;
  const maxhold = document.getElementById('maxHoldMs').value;
  const maxrecover = document.getElementById('maxRecoverMs').value;

  fetch(`/setmission?deep=${encodeURIComponent(deep)}&shallow=${encodeURIComponent(shallow)}&surface=${encodeURIComponent(surface)}&hold=${encodeURIComponent(hold)}&tol=${encodeURIComponent(tol)}&stol=${encodeURIComponent(stol)}&maxtransit=${encodeURIComponent(maxtransit)}&maxhold=${encodeURIComponent(maxhold)}&maxrecover=${encodeURIComponent(maxrecover)}`)
    .then(res => res.json())
    .then(data => {
      document.getElementById('missionStatus').innerText =
        `Mission: deep=${data.deep_m} m, shallow=${data.shallow_m} m, hold=${data.hold_ms} ms`;
    });
}

function drawGraph(times, depths) {
  const canvas = document.getElementById('graph');
  const ctx = canvas.getContext('2d');

  ctx.clearRect(0, 0, canvas.width, canvas.height);

  if (!times || times.length < 2) {
    ctx.fillStyle = '#000';
    ctx.font = '18px Helvetica';
    ctx.fillText('Not enough data to graph', 20, 30);
    return;
  }

  const left = 70;
  const right = canvas.width - 20;
  const top = 20;
  const bottom = canvas.height - 50;
  const width = right - left;
  const height = bottom - top;

  const tMin = times[0] / 1000.0;
  const tMax = times[times.length - 1] / 1000.0;

  let dMin = depths[0];
  let dMax = depths[0];
  for (let i = 1; i < depths.length; i++) {
    if (depths[i] < dMin) dMin = depths[i];
    if (depths[i] > dMax) dMax = depths[i];
  }

  if (dMax === dMin) {
    dMax += 0.1;
    dMin -= 0.1;
  }

  // Add padding to depth range
  const pad = (dMax - dMin) * 0.1;
  dMax += pad;
  dMin -= pad;

  // Prevent weird values
  if (dMin < 0) dMin = 0;

  // Background
  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Axes
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(left, top);
  ctx.lineTo(left, bottom);
  ctx.lineTo(right, bottom);
  ctx.stroke();

  // ----- Grid and Y ticks -----
  const yTicks = 6;
  ctx.font = '12px Helvetica';
  ctx.fillStyle = '#000';
  ctx.strokeStyle = '#ccc';
  ctx.lineWidth = 1;

  for (let i = 0; i <= yTicks; i++) {
    const frac = i / yTicks;
    const y = top + frac * height;

    // Because deeper should be lower on screen:
    const depthValue = dMin + frac * (dMax - dMin);

    // horizontal grid line
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
    ctx.stroke();

    // tick mark
    ctx.strokeStyle = '#000';
    ctx.beginPath();
    ctx.moveTo(left - 6, y);
    ctx.lineTo(left, y);
    ctx.stroke();

    // label
    ctx.fillText(depthValue.toFixed(2), 10, y + 4);

    ctx.strokeStyle = '#ccc';
  }

  // ----- Grid and X ticks -----
  const xTicks = 6;
  for (let i = 0; i <= xTicks; i++) {
    const frac = i / xTicks;
    const x = left + frac * width;
    const timeValue = tMin + frac * (tMax - tMin);

    // vertical grid line
    ctx.beginPath();
    ctx.moveTo(x, top);
    ctx.lineTo(x, bottom);
    ctx.stroke();

    // tick mark
    ctx.strokeStyle = '#000';
    ctx.beginPath();
    ctx.moveTo(x, bottom);
    ctx.lineTo(x, bottom + 6);
    ctx.stroke();

    // label
    ctx.fillStyle = '#000';
    ctx.fillText(timeValue.toFixed(1), x - 10, bottom + 20);

    ctx.strokeStyle = '#ccc';
  }

  // Axis labels
  ctx.fillStyle = '#000';
  ctx.font = '14px Helvetica';
  ctx.fillText('Depth (m)', 10, 15);
  ctx.fillText('Time (s)', right - 55, canvas.height - 10);

  // ----- Plot -----
  ctx.strokeStyle = '#0077cc';
  ctx.lineWidth = 2;
  ctx.beginPath();

  for (let i = 0; i < times.length; i++) {
    const t = times[i] / 1000.0;
    const d = depths[i];

    const x = left + ((t - tMin) / (tMax - tMin)) * width;

    // IMPORTANT:
    // shallow depth near top, deeper depth near bottom
    const y = top + ((d - dMin) / (dMax - dMin)) * height;

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }

  ctx.stroke();
}

function loadPID() {
  fetch('/pid')
    .then(res => res.json())
    .then(data => {
      document.getElementById('kp').value = data.kp;
      document.getElementById('ki').value = data.ki;
      document.getElementById('kd').value = data.kd;
      document.getElementById('pidStatus').innerText =
        `PID: Kp=${data.kp}, Ki=${data.ki}, Kd=${data.kd}`;
    });
}

function applyPID() {
  const kp = document.getElementById('kp').value;
  const ki = document.getElementById('ki').value;
  const kd = document.getElementById('kd').value;

  fetch(`/setpid?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}`)
    .then(res => res.json())
    .then(data => {
      document.getElementById('pidStatus').innerText =
        `PID: Kp=${data.kp}, Ki=${data.ki}, Kd=${data.kd}`;
    });
}

setInterval(updateElapsed, 250);
setInterval(updateRTC, 1000);
setInterval(updatePressure, 300);
setInterval(updateStatus, 500);

clearGraph();
updateElapsed();
updateRTC();
updatePressure();
updateStatus();
loadPID();
loadManual();
loadMission();
</script>

</body>
</html>
)rawliteral");
}
