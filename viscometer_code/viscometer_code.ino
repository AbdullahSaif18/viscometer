// viscometer_updated.ino
// ESP32 Falling Ball Viscometer - LDR-based warning, SH1106/U8g2 (flipped 180°), WebSocket UI (SPIFFS)
// Changes:
// - Replaced DS18B20 with DHT11 (any pin; default DHT_PIN = 4)
// - Dynamic glycerin density: rho(T) = 1270 - 0.612 * T (kg/m^3)
// - Updated default constants to match user inputs
// - Display rotated 180° (U8G2_R2)
// - Viscosity label "Viscosity (μ):" and value printed larger

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <DHT.h>

/* ---------- PINOUT ---------- */
#define SENSOR1_PIN      15
#define SENSOR2_PIN      2
#define LASER_PIN        18
#define BUZZER_PIN       19
#define DHT_PIN          4   // default: you said "any pin" - change if needed

/* ---------- DHT ---------- */
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
bool haveDHT = false;

/* ---------- Display (SH1106 with U8g2) ---------- */
/* flipped 180 degrees */
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/21);
bool displayOk = false;

/* ---------- Constants / Defaults (from user) ---------- */
const float DEFAULT_SENSOR_DISTANCE = 0.096f;   // 9.6 cm
const float DEFAULT_TUBE_DIAMETER   = 0.04f;    // 4.0 cm
const float DEFAULT_GRAVITY         = 9.8f;     // 9.8 m/s^2
const float BALL_DIAMETER_DEFAULT   = 0.0254f;  // 1 inch = 25.4 mm
const float BALL_MASS_DEFAULT       = 0.056f;   // 56 g
const float BALL_DENSITY_DEFAULT    = 6520.0f;  // approximate steel ball density
const float LIQUID_DENSITY_DEFAULT  = 1260.0f;  // glycerin at 20°C (fallback)

/* Warning timeout (ms) */
const unsigned long WARNING_TIMEOUT_MS = 2000UL; // 2 seconds

/* Buzzer warning cadence */
const unsigned long WARNING_BEEP_INTERVAL_MS = 600UL;
const int WARNING_BEEP_MS = 150;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/* ---------- Timing & ISR globals ---------- */
volatile uint32_t t1_us = 0;
volatile uint32_t t2_us = 0;
volatile bool got_t1 = false;
volatile bool got_t2 = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

unsigned long runCounter = 1;
bool laserOn = false;
bool armed = true;

/* Averaging */
bool averagingMode = false;
int averagingCount = 5;
int averagingProgress = 0;
float averagingTempSum = 0.0f;
float averagingViscSum = 0.0f;

float lastTempC = NAN;
float lastTimeS = 0.0f;
float lastVisc = 0.0f;

/* Settings struct */
struct Settings {
  bool isCustom = false;
  float ballDensity = BALL_DENSITY_DEFAULT;
  float ballDiameter = BALL_DIAMETER_DEFAULT;
  float liquidDensity = LIQUID_DENSITY_DEFAULT; // initial default; overridden by temp-based calc
  float tubeInnerDiameter = DEFAULT_TUBE_DIAMETER;
  float sensorDistance = DEFAULT_SENSOR_DISTANCE;
  float gravity = DEFAULT_GRAVITY;
  int dhtPin = DHT_PIN;
} settings;

/* Warning state */
unsigned long lastSeenMillis1 = 0;
unsigned long lastSeenMillis2 = 0;
bool warning1Active = false;
bool warning2Active = false;
unsigned long lastWarningBeep = 0;

/* Forward declarations */
void IRAM_ATTR sensor1ISR();
void IRAM_ATTR sensor2ISR();
float computeViscosity(float time_s, float tempC);
float wallCorrectionFactor(float a, float R);
float computeReynolds(float mu, float v, float rho_f, float diameter);
void sendSettingsToAll();
void sendUpdateToAll();
void sendLogToAll(float temp, float time_s, float visc);
void writeRunCsv(unsigned long runNo, float temp, float t, float visc);
void beepOnceMs(int ms);
void beepPatternWarning();

void beepOnceMs(int ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN, LOW);
}
void beepPatternWarning() {
  unsigned long now = millis();
  if (now - lastWarningBeep >= WARNING_BEEP_INTERVAL_MS) {
    lastWarningBeep = now;
    digitalWrite(BUZZER_PIN, HIGH);
    delay(WARNING_BEEP_MS);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);
  delay(50);

  if (!SPIFFS.begin(true)) Serial.println("SPIFFS failed");

  /* WiFi AP (keeps your web UI accessible locally) */
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Viscometer", "viscometer123");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  /* DHT */
  dht.begin();
  haveDHT = true; // we assume DHT11 is present; if not, read will return NAN/invalid

  /* Display */
  display.begin();
  displayOk = true;
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(0, 12, "VISCOMETER");
  display.drawStr(0, 30, "Initializing...");
  display.sendBuffer();
  delay(600);

  /* Pins and interrupts */
  pinMode(SENSOR1_PIN, INPUT_PULLUP);
  pinMode(SENSOR2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR1_PIN), sensor1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SENSOR2_PIN), sensor2ISR, FALLING);

  pinMode(LASER_PIN, OUTPUT); digitalWrite(LASER_PIN, LOW); laserOn = false;
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);

  /* Default settings */
  settings = Settings();

  /* Web server + websocket */
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/runs.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    if (SPIFFS.exists("/runs.csv")) request->send(SPIFFS, "/runs.csv", "text/csv");
    else request->send(404, "text/plain", "No runs recorded.");
  });

  server.on("/runs.csv", HTTP_DELETE, [](AsyncWebServerRequest *request){
    if (SPIFFS.exists("/runs.csv")) {
      SPIFFS.remove("/runs.csv");
      request->send(200, "text/plain", "Deleted");
    } else {
      request->send(404, "text/plain", "No runs recorded.");
    }
  });

  ws.onEvent([](AsyncWebSocket *serverPtr, AsyncWebSocketClient *client, AwsEventType type,
                void *arg, uint8_t *data, size_t len){
    if (type == WS_EVT_CONNECT) {
      Serial.printf("WS client #%u connected\n", client->id());
      sendSettingsToAll();
      sendUpdateToAll();
    }
    if (type == WS_EVT_DATA) {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (!info->final || info->opcode != WS_TEXT) return;

      String msg = "";
      for (size_t i=0;i<len;i++) msg += (char)data[i];

      StaticJsonDocument<512> doc;
      auto err = deserializeJson(doc, msg);
      if (err) {
        Serial.println("Invalid JSON from client");
        return;
      }
      const char* t = doc["type"];
      if (!t) return;
      if (strcmp(t, "getSettings") == 0) {
        sendSettingsToAll();
      } else if (strcmp(t, "useDefaults") == 0) {
        settings = Settings();
        sendSettingsToAll();
      } else if (strcmp(t, "setSettings") == 0) {
        if (doc.containsKey("ballDensity")) settings.ballDensity = doc["ballDensity"];
        if (doc.containsKey("ballDiameter")) settings.ballDiameter = doc["ballDiameter"];
        if (doc.containsKey("liquidDensity")) settings.liquidDensity = doc["liquidDensity"];
        if (doc.containsKey("tubeDiameter")) settings.tubeInnerDiameter = doc["tubeDiameter"];
        if (doc.containsKey("sensorDistance")) settings.sensorDistance = doc["sensorDistance"];
        if (doc.containsKey("gravity")) settings.gravity = doc["gravity"];
        if (doc.containsKey("dhtPin")) settings.dhtPin = doc["dhtPin"];
        settings.isCustom = true;
        sendSettingsToAll();
      } else if (strcmp(t, "startMeasurement") == 0) {
        armed = true;
        digitalWrite(LASER_PIN, HIGH);
        laserOn = true;
      } else if (strcmp(t, "stopMeasurement") == 0) {
        armed = false;
        digitalWrite(LASER_PIN, LOW);
        laserOn = false;
      } else if (strcmp(t, "laserOn") == 0) {
        digitalWrite(LASER_PIN, HIGH);
        laserOn = true;
      } else if (strcmp(t, "laserOff") == 0) {
        digitalWrite(LASER_PIN, LOW);
        laserOn = false;
      } else if (strcmp(t, "setAveraging") == 0) {
        averagingMode = doc["enabled"] | false;
        averagingCount = doc["count"] | 5;
      }
    }
  });
  server.addHandler(&ws);

  server.begin();
  Serial.println("Server started (SPIFFS index.html served)");
}

/* ---------- ISRs ---------- */
void IRAM_ATTR sensor1ISR() {
  if (!armed) return;
  portENTER_CRITICAL_ISR(&mux);
  t1_us = micros();
  got_t1 = true;
  got_t2 = false;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR sensor2ISR() {
  if (!armed) return;
  portENTER_CRITICAL_ISR(&mux);
  if (got_t1) {
    t2_us = micros();
    got_t2 = true;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

/* ---------- Helpers ---------- */
void sendSettingsToAll() {
  StaticJsonDocument<256> doc;
  doc["type"] = "settings";
  doc["ballDensity"] = settings.ballDensity;
  doc["ballDiameter"] = settings.ballDiameter;
  doc["liquidDensity"] = settings.liquidDensity;
  doc["tubeDiameter"] = settings.tubeInnerDiameter;
  doc["sensorDistance"] = settings.sensorDistance;
  doc["gravity"] = settings.gravity;
  doc["isCustom"] = settings.isCustom;
  doc["dhtPin"] = settings.dhtPin;
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

void sendUpdateToAll() {
  StaticJsonDocument<256> doc;
  doc["type"] = "update";
  doc["temp"] = lastTempC;
  doc["time"] = lastTimeS;
  doc["visc"] = lastVisc;
  doc["sensor1"] = digitalRead(SENSOR1_PIN) == LOW;
  doc["sensor2"] = digitalRead(SENSOR2_PIN) == LOW;
  doc["warning1"] = warning1Active;
  doc["warning2"] = warning2Active;
  doc["laserOn"] = laserOn;
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

void sendLogToAll(float temp, float time_s, float visc) {
  StaticJsonDocument<256> doc;
  doc["type"] = "log";
  doc["temp"] = temp;
  doc["time"] = time_s;
  doc["visc"] = visc;
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

void writeRunCsv(unsigned long runNo, float temp, float t, float visc) {
  File f = SPIFFS.open("/runs.csv", FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open runs.csv");
    return;
  }
  char buf[128];
  int n = snprintf(buf, sizeof(buf), "%lu,%.2f,%.6f,%.9g\n", runNo, temp, t, visc);
  f.write((const uint8_t*)buf, n);
  f.close();
}

/* ---------- Physics ---------- */
/* wall correction factor (empirical) */
float wallCorrectionFactor(float a, float R) {
  float lambda = a / max(1e-9f, R);
  float f = 1.0f + 2.4f * lambda + 3.3f * lambda * lambda * lambda;
  return constrain(f, 1.0f, 10.0f);
}

/* compute viscosity using Stokes' law with wall correction
   time_s = fall time between sensors
   tempC = fluid temperature (C) used to compute dynamic glycerin density
*/
float computeViscosity(float time_s, float tempC) {
  float v = settings.sensorDistance / max(time_s, 1e-9f); // m/s
  float a = settings.ballDiameter * 0.5f; // radius (m)
  float rho_s = settings.ballDensity; // ball density (kg/m^3)

  // dynamic fluid density for glycerin (kg/m^3)
  // formula: rho(T) = 1270 - 0.612 * T  (valid around 0-100°C approximate)
  float rho_f;
  if (!isnan(tempC)) {
    rho_f = 1270.0f - 0.612f * tempC;
  } else {
    rho_f = settings.liquidDensity; // fallback
  }

  float g = settings.gravity;

  // base Stokes formula for sphere at terminal velocity:
  // mu = (2/9) * a^2 * g * (rho_s - rho_f) / v
  float mu = (2.0f / 9.0f) * (a * a) * g * (rho_s - rho_f) / max(v, 1e-9f);

  // apply wall correction
  float R = settings.tubeInnerDiameter * 0.5f;
  mu *= wallCorrectionFactor(a, R);

  // enforce non-negative
  if (mu <= 0.0f || isnan(mu) || isinf(mu)) mu = 0.0f;
  return mu;
}

/* Reynolds number */
float computeReynolds(float mu, float v, float rho_f, float diameter) {
  if (mu <= 0) return 9999.0f;
  return rho_f * v * diameter / mu;
}

/* ---------- Main loop ---------- */
unsigned long lastTempMillis = 0;
unsigned long lastBroadcast = 0;
unsigned long lastDisplay = 0;

void loop() {
  // read DHT11 every 800 ms
  if (millis() - lastTempMillis > 800) {
    lastTempMillis = millis();
    if (haveDHT) {
      float t = dht.readTemperature(); // Celsius
      if (isnan(t)) {
        // failed read, leave lastTempC unchanged or set to 0
        Serial.println("DHT read failed");
      } else {
        lastTempC = t;
      }
    } else {
      lastTempC = NAN;
    }
  }

  // Poll LDRs frequently to update lastSeenMillis
  if (digitalRead(SENSOR1_PIN) == LOW) {
    lastSeenMillis1 = millis();
  }
  if (digitalRead(SENSOR2_PIN) == LOW) {
    lastSeenMillis2 = millis();
  }

  // Check warnings (only when laser is ON)
  warning1Active = false;
  warning2Active = false;
  if (laserOn) {
    if (millis() - lastSeenMillis1 > WARNING_TIMEOUT_MS) warning1Active = true;
    if (millis() - lastSeenMillis2 > WARNING_TIMEOUT_MS) warning2Active = true;
  }

  if (warning1Active || warning2Active) {
    beepPatternWarning();
  }

  // handle completed run if both timestamps set by ISRs
  bool finish = false;
  uint32_t a_us = 0, b_us = 0;
  portENTER_CRITICAL(&mux);
  if (got_t1 && got_t2) {
    a_us = t1_us;
    b_us = t2_us;
    got_t1 = false;
    got_t2 = false;
    finish = true;
  }
  portEXIT_CRITICAL(&mux);

  if (finish) {
    // quick beep for top sensor detection
    beepOnceMs(40);

    float dt = (b_us > a_us) ? ((b_us - a_us) / 1e6f) : 0.0f;
    lastTimeS = dt;

    // update dynamic liquid density in settings for UI visibility
    if (!isnan(lastTempC)) {
      settings.liquidDensity = 1270.0f - 0.612f * lastTempC;
    }

    lastVisc = computeViscosity(dt, lastTempC);

    // Reynolds check
    float v = settings.sensorDistance / max(dt, 1e-9f);
    float Re = computeReynolds(lastVisc, v, settings.liquidDensity, settings.ballDiameter);
    if (Re > 1.0f) {
      // warn that Stokes assumptions may not hold
      for (int i = 0; i < 3; i++) { beepOnceMs(80); delay(60); }
    } else {
      // normal double beep for completion
      beepOnceMs(60); delay(60); beepOnceMs(60);
    }

    // log & broadcast
    writeRunCsv(runCounter, lastTempC, lastTimeS, lastVisc);
    sendLogToAll(lastTempC, lastTimeS, lastVisc);
    sendUpdateToAll();
    runCounter++;

    // averaging support
    if (averagingMode) {
      averagingProgress++;
      averagingTempSum += lastTempC;
      averagingViscSum += lastVisc;
      if (averagingProgress >= averagingCount) {
        float avgTemp = averagingTempSum / averagingProgress;
        float avgVisc = averagingViscSum / averagingProgress;
        sendLogToAll(avgTemp, 0.0, avgVisc); // time==0 indicates averaged entry
        averagingMode = false;
        averagingProgress = 0;
        averagingTempSum = 0;
        averagingViscSum = 0;
      }
    }
  }

  // periodic websocket broadcast
  if (millis() - lastBroadcast > 400) {
    lastBroadcast = millis();
    sendUpdateToAll();
  }

  // OLED update
  if (displayOk && millis() - lastDisplay > 250) {
    lastDisplay = millis();
    display.clearBuffer();

    // small font for temperature/time
    display.setFont(u8g2_font_6x12_tf);
    char buf[64];
    if (!isnan(lastTempC)) {
      snprintf(buf, sizeof(buf), "T: %.2f C", lastTempC);
    } else {
      snprintf(buf, sizeof(buf), "T: -- C");
    }
    display.drawStr(0, 12, buf);
    snprintf(buf, sizeof(buf), "t: %.6f s", lastTimeS);
    display.drawStr(0, 28, buf);

    // Viscosity label and large value
    display.setFont(u8g2_font_ncenB14_tr); // larger font for viscosity value
    display.drawStr(0, 48, "Viscosity:");
    // Draw value to the right (formatted)
    char vbuf[32];
    if (lastVisc > 0.0f) {
      // show in Pa.s
      snprintf(vbuf, sizeof(vbuf), "%.5g Pa.s", lastVisc);
    } else {
      snprintf(vbuf, sizeof(vbuf), "-- Pa.s");
    }
    // draw value under the label if it doesn't fit
    display.setCursor(0, 65);
    display.drawStr(0, 65, vbuf);

    // warning area (blinking)
    static bool blink = false;
    blink = !blink;
    display.setFont(u8g2_font_6x12_tf);
    if (warning1Active || warning2Active) {
      if (blink) {
        if (warning1Active) display.drawStr(90, 12, "⚠S1");
        if (warning2Active) display.drawStr(90, 28, "⚠S2");
      }
    } else {
      if (laserOn) display.drawStr(90, 12, "ON");
      else display.drawStr(90, 12, "OFF");
    }

    display.sendBuffer();
  }

  delay(5);
}
