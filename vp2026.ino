#include <WiFi.h>
#include <Wire.h>
#include <RTClib.h>
#include <MS5837.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ===== Team / mission info =====
const char* TEAM_ID = "EX01";

// ===== Mission options =====
const bool SELF_RECOVER_TO_SURFACE = false;   // false = stay at 0.40 m, true = surface after final hold

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
const int ACTUATOR_MIN_US = 1000;
const int ACTUATOR_MAX_US = 2000;
const int ACTUATOR_NEUTRAL_US = 1500;
const int ACTUATOR_IDLE_US = 1000;

// Startup priming
const int PRIME_HIGH_US = 2000;
const int PRIME_LOW_US = 1000;
const unsigned long PRIME_HIGH_TIME_MS = 1500;
const unsigned long PRIME_LOW_TIME_MS = 1500;
const unsigned long PRIME_SETTLE_TIME_MS = 500;

// If larger PWM makes the float go deeper, leave this as +1.
// If it goes the wrong way, change to -1.
const int CONTROL_DIRECTION = -1;

int actuatorCommandUs = -1;

// ===== PID tuning =====
// Output is in microseconds
float PID_KP = 220.0f;
float PID_KI = 4.0f;
float PID_KD = 90.0f;

// Integral clamp to prevent windup
float PID_INTEGRAL_MIN = -1.5f;
float PID_INTEGRAL_MAX =  1.5f;

// PID update rate
const unsigned long PID_INTERVAL_MS = 100;

// ===== Mission targets =====
const float TARGET_DEEP_M = 2.50f;
const float TARGET_SHALLOW_M = 0.40f;
const float TARGET_SURFACE_M = 0.02f;         // near-surface recovery target
const unsigned long HOLD_TIME_MS = 30000;
const float TARGET_TOLERANCE_M = 0.05f;       // enter hold when within ±5 cm
const float SURFACE_TOLERANCE_M = 0.05f;

// ===== Zeroing =====
const int ZERO_SAMPLE_COUNT = 20;
const unsigned long ZERO_SAMPLE_DELAY_MS = 50;
float depthZeroOffset_m = 0.0f;

// ===== NeoPixel Jewel =====
const int PIXEL_PIN = 10;
const int PIXEL_COUNT = 7;
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

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
  RECOVER_SURFACE
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

void resetPID();
void runDepthPID(float targetDepthM);
void zeroDepthSensor();
void logSampleIfNeeded();

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
}

// ===== State helpers =====
void enterState(State newState) {
  currentState = newState;
  stateEntryMillis = millis();
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
  uint32_t color;

  switch (currentState) {
    case IDLE:
      color = pixels.Color(0, 0, 255);       // blue
      break;

    case DESCEND_1:
    case DESCEND_2:
      color = pixels.Color(255, 0, 255);     // magenta
      break;

    case HOLD_250_1:
    case HOLD_250_2:
      color = pixels.Color(255, 0, 0);       // red
      break;

    case ASCEND_1:
    case ASCEND_2:
      color = pixels.Color(255, 180, 0);     // amber
      break;

    case HOLD_040_1:
    case HOLD_040_2:
      color = pixels.Color(0, 255, 0);       // green
      break;

    case STATION_KEEP_040:
      color = pixels.Color(255, 255, 255);   // white
      break;

    case RECOVER_SURFACE:
      color = pixels.Color(0, 255, 255);     // cyan
      break;

    default:
      color = pixels.Color(0, 0, 0);
      break;
  }

  for (int i = 0; i < PIXEL_COUNT; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

// ===== Sensor update =====
void updateSensors() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();

    pressureSensor.read();
    livePressure_mbar = pressureSensor.pressure();
    liveTemp_C = pressureSensor.temperature();
    rawDepth_m = pressureSensor.depth();
    liveDepth_m = rawDepth_m - depthZeroOffset_m;
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

  for (int i = 0; i < ZERO_SAMPLE_COUNT; i++) {
    pressureSensor.read();
    sum += pressureSensor.depth();
    delay(ZERO_SAMPLE_DELAY_MS);
  }

  depthZeroOffset_m = sum / ZERO_SAMPLE_COUNT;

  Serial.print("Depth zero offset (m): ");
  Serial.println(depthZeroOffset_m, 4);

  pressureSensor.read();
  rawDepth_m = pressureSensor.depth();
  liveDepth_m = rawDepth_m - depthZeroOffset_m;
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

// ===== Mission control =====
void startMission() {
  sampleCount = 0;
  loggingEnabled = true;
  loggingStartMillis = millis();
  lastSampleMillis = 0;

  zeroDepthSensor();
  enterState(DESCEND_1);

  Serial.println("Mission started");
}

void stopLoggingAndIdle() {
  loggingEnabled = false;
  enterState(IDLE);
  setActuatorUs(ACTUATOR_IDLE_US);

  Serial.println("Logging stopped / returned to IDLE");
  Serial.print("Samples logged: ");
  Serial.println(sampleCount);
}

// ===== State Machine =====
void runStateMachine() {
  switch (currentState) {
    case IDLE:
      setActuatorUs(ACTUATOR_IDLE_US);
      break;

    case DESCEND_1:
      runDepthPID(TARGET_DEEP_M);
      if (liveDepth_m >= (TARGET_DEEP_M - TARGET_TOLERANCE_M)) {
        enterState(HOLD_250_1);
      }
      break;

    case HOLD_250_1:
      runDepthPID(TARGET_DEEP_M);
      if (millis() - stateEntryMillis >= HOLD_TIME_MS) {
        enterState(ASCEND_1);
      }
      break;

    case ASCEND_1:
      runDepthPID(TARGET_SHALLOW_M);
      if (liveDepth_m <= (TARGET_SHALLOW_M + TARGET_TOLERANCE_M)) {
        enterState(HOLD_040_1);
      }
      break;

    case HOLD_040_1:
      runDepthPID(TARGET_SHALLOW_M);
      if (millis() - stateEntryMillis >= HOLD_TIME_MS) {
        enterState(DESCEND_2);
      }
      break;

    case DESCEND_2:
      runDepthPID(TARGET_DEEP_M);
      if (liveDepth_m >= (TARGET_DEEP_M - TARGET_TOLERANCE_M)) {
        enterState(HOLD_250_2);
      }
      break;

    case HOLD_250_2:
      runDepthPID(TARGET_DEEP_M);
      if (millis() - stateEntryMillis >= HOLD_TIME_MS) {
        enterState(ASCEND_2);
      }
      break;

    case ASCEND_2:
      runDepthPID(TARGET_SHALLOW_M);
      if (liveDepth_m <= (TARGET_SHALLOW_M + TARGET_TOLERANCE_M)) {
        enterState(HOLD_040_2);
      }
      break;

    case HOLD_040_2:
      runDepthPID(TARGET_SHALLOW_M);
      if (millis() - stateEntryMillis >= HOLD_TIME_MS) {
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
      if (liveDepth_m <= (TARGET_SURFACE_M + SURFACE_TOLERANCE_M)) {
        setActuatorUs(ACTUATOR_IDLE_US);
      }
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
  <div class="med" id="rtc">RTC: --</div>
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

  const left = 60;
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

  const pad = (dMax - dMin) * 0.1;
  dMax += pad;
  dMin -= pad;

  ctx.strokeStyle = '#000';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(left, top);
  ctx.lineTo(left, bottom);
  ctx.lineTo(right, bottom);
  ctx.stroke();

  ctx.fillStyle = '#000';
  ctx.font = '14px Helvetica';
  ctx.fillText('Depth (m)', 10, 15);
  ctx.fillText('Time (s)', right - 60, canvas.height - 10);

  ctx.fillText(dMax.toFixed(3), 5, top + 5);
  ctx.fillText(dMin.toFixed(3), 5, bottom);

  ctx.fillText(tMin.toFixed(1), left - 5, bottom + 20);
  ctx.fillText(tMax.toFixed(1), right - 20, bottom + 20);

  ctx.strokeStyle = '#0077cc';
  ctx.lineWidth = 2;
  ctx.beginPath();

  for (let i = 0; i < times.length; i++) {
    const t = times[i] / 1000.0;
    const d = depths[i];

    const x = left + ((t - tMin) / (tMax - tMin)) * width;
    const y = bottom - ((d - dMin) / (dMax - dMin)) * height;

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
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
</script>

</body>
</html>
)rawliteral");
}