#include <WiFi.h>
#include <Wire.h>
#include <RTClib.h>
#include <MS5837.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>

// ===== Team / mission info =====
const char* TEAM_ID = "EX01";

// ===== WiFi AP config =====
const char* ssid = "VP_Float";

WiFiServer server(80);

// Static IP
IPAddress local_IP(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
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

// Real PWM pulse widths for more granular control
const int ACTUATOR_IDLE_US = 1000;
const int ACTUATOR_DIVE_US = 2000;

int actuatorCommandUs = -1;

// ===== NeoPixel Jewel =====
const int PIXEL_PIN = 10;
const int PIXEL_COUNT = 7;
Adafruit_NeoPixel pixels(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ===== State machine =====
enum State {
  IDLE,
  LOGGING,
  DONE
};

State currentState = IDLE;

// ===== Logging =====
const int MAX_SAMPLES = 2000;                 // about 400 seconds at 5 Hz
unsigned long timeLog[MAX_SAMPLES];           // ms since start
float depthLog[MAX_SAMPLES];                  // depth in meters
int sampleCount = 0;

unsigned long loggingStartMillis = 0;
unsigned long lastSampleMillis = 0;
const unsigned long SAMPLE_INTERVAL_MS = 200; // 5 Hz logging

// ===== Live values =====
float livePressure_mbar = 0.0;
float liveTemp_C = 0.0;
float liveDepth_m = 0.0;

// ===== RTC display =====
DateTime nowRTC;

// ===== Function declarations =====
void handleClient();
void runStateMachine();
void updateLivePressure();
void startLogging();
void stopLogging();
void setActuatorUs(int pulseUs);
void updateStateLEDs();

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

  // I2C start on known-good pins
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

  // Bar02 uses the MS5837-02BA model
  pressureSensor.setModel(MS5837::MS5837_02BA);

  // Fresh water density
  pressureSensor.setFluidDensity(997);

  // First read to initialize live values
  pressureSensor.read();
  livePressure_mbar = pressureSensor.pressure();
  liveTemp_C = pressureSensor.temperature();
  liveDepth_m = pressureSensor.depth();

  // ----- Buoyancy actuator -----
  buoyancyServo.setPeriodHertz(50);
  buoyancyServo.attach(ACTUATOR_PIN, 500, 2500);
  setActuatorUs(ACTUATOR_IDLE_US);
  Serial.println("Servo attached.");

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
  runStateMachine();
  updateLivePressure();
}

// ===== Actuator helper =====
void setActuatorUs(int pulseUs) {
  if (pulseUs != actuatorCommandUs) {
    actuatorCommandUs = pulseUs;
    buoyancyServo.writeMicroseconds(actuatorCommandUs);

    Serial.print("Actuator command: ");
    Serial.print(actuatorCommandUs);
    Serial.println(" us");
  }
}

// ===== NeoPixel state helper =====
void updateStateLEDs() {
  uint32_t color;

  switch (currentState) {
    case IDLE:
      color = pixels.Color(0, 0, 255);   // blue
      break;

    case LOGGING:
      color = pixels.Color(0, 255, 0);   // green
      break;

    case DONE:
      color = pixels.Color(255, 0, 0);   // red
      break;

    default:
      color = pixels.Color(0, 0, 0);     // off
      break;
  }

  for (int i = 0; i < PIXEL_COUNT; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

// ===== Live sensor refresh =====
void updateLivePressure() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 250) {
    lastUpdate = millis();

    pressureSensor.read();
    livePressure_mbar = pressureSensor.pressure();
    liveTemp_C = pressureSensor.temperature();
    liveDepth_m = pressureSensor.depth();
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
    startLogging();
    sendOK(client, "STARTED");
  }
  else if (request.indexOf("GET /stop") >= 0) {
    stopLogging();
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

// ===== State control =====
void startLogging() {
  sampleCount = 0;
  loggingStartMillis = millis();
  lastSampleMillis = 0;
  currentState = LOGGING;
  updateStateLEDs();
  setActuatorUs(ACTUATOR_DIVE_US);
  Serial.println("Logging started");
}

void stopLogging() {
  if (currentState == LOGGING) {
    currentState = DONE;
    updateStateLEDs();
    setActuatorUs(ACTUATOR_IDLE_US);
    Serial.println("Logging stopped");
    Serial.print("Samples logged: ");
    Serial.println(sampleCount);
  }
}

// ===== State Machine =====
void runStateMachine() {
  switch (currentState) {

    case IDLE:
      setActuatorUs(ACTUATOR_IDLE_US);
      break;

    case LOGGING:
      setActuatorUs(ACTUATOR_DIVE_US);

      if (millis() - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
        lastSampleMillis = millis();

        if (sampleCount < MAX_SAMPLES) {
          pressureSensor.read();
          timeLog[sampleCount] = millis() - loggingStartMillis;
          depthLog[sampleCount] = pressureSensor.depth();
          sampleCount++;
        }
      }
      break;

    case DONE:
      setActuatorUs(ACTUATOR_IDLE_US);
      break;
  }
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

  if (currentState == LOGGING) {
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
  if (currentState == IDLE) client.print("IDLE");
  else if (currentState == LOGGING) client.print("LOGGING");
  else client.print("DONE");
  client.print("\",");

  client.print("\"team_id\":\"");
  client.print(TEAM_ID);
  client.print("\",");

  client.print("\"samples\":");
  client.print(sampleCount);
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
  max-width: 700px;
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

  // axes
  ctx.strokeStyle = '#000';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(left, top);
  ctx.lineTo(left, bottom);
  ctx.lineTo(right, bottom);
  ctx.stroke();

  // labels
  ctx.fillStyle = '#000';
  ctx.font = '14px Helvetica';
  ctx.fillText('Depth (m)', 10, 15);
  ctx.fillText('Time (s)', right - 60, canvas.height - 10);

  ctx.fillText(dMax.toFixed(3), 5, top + 5);
  ctx.fillText(dMin.toFixed(3), 5, bottom);

  ctx.fillText(tMin.toFixed(1), left - 5, bottom + 20);
  ctx.fillText(tMax.toFixed(1), right - 20, bottom + 20);

  // plot
  ctx.strokeStyle = '#0077cc';
  ctx.lineWidth = 2;
  ctx.beginPath();

  for (let i = 0; i < times.length; i++) {
    const t = times[i] / 1000.0;
    const d = depths[i];

    const x = left + ((t - tMin) / (tMax - tMin)) * width;
    const y = bottom - ((d - dMin) / (dMax - dMin)) * height;

    if (i == 0) ctx.moveTo(x, y);
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