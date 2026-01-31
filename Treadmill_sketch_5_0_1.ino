/*******************************************************
 * ESP32 FUTÓPAD ADATGYŰJTŐ ÉS WEBSZERVER  5_0_1
 *
 * Mér:
 *  - Sebesség (hall szenzor – futószalag fordulat)
 *  - Szalagsebesség (opto – tényleges szalag mozgás)
 *  - Csúszás (%)
 *  - Dőlésszög (MPU6050 – gyorsulásból)
 *  - Lépésszám (MPU6050 Z tengely – futó által keltett rezgés)
 *  - Pulzus (BLE HeartRate)
 *
 * Megjelenítés:
 *  - Webes felületen (index.html LittleFS-ből)
 *  - Valós idejű grafikon
 *
 * Fájlkezelés:
 *  - Listázás
 *  - Letöltés
 *  - Egyedi törlés
 *  - Összes törlés (index.html kivétel)
 *  - Start/Stop rögzítés (webes gombok)
 *******************************************************/

#include <Wire.h>
#include <MPU6050.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "netdef.h" //Az alábbi 4 darab hálózati azonosítók definíciós file-ja.
//#define WIFI_SSID 
//#define WIFI_PASSWORD 
//#define THINGSPEAK_API_KEY   
//#define YOUR_CHANNEL_ID 

#include "BLE_HEARTRATE.h"
#include "TREADMILL_LOGGER.h"

// ================== HARDVER KONFIG ==================
#define HALL_SENSOR_PIN 15  //szalagmozgató orsón levő szenzor 1 impulzus fordulatonként. Orsó átmérő 49mm
#define BELT_DISTANCE 15.25 //két impulzus közti szalagmozgás cm
#define HALL_DEBOUNCE_US 15000  //15ms  not implemented yet

#define OPTO_SENSOR_PIN 27 //szalag 1 körbefordulásának detektálása
#define BELT_LENGTH_M 2.85
#define OPTO_DEBOUNCE_US 300000  //300ms

#define RAD2DEG 57.29578

// ================== SZŰRÉSI PARAMÉTEREK ==================
#define MEDIANFILTER_FOR_TILT 41
#define MEDIANFILTER_FOR_STEP 11

float ALPHA_TILT  = 0.01;
float ALPHA_SPEED = 0.3;

// Lépésdetektálás paraméterek
float STEP_THRESHOLD_HIGH = 1.13;
float STEP_THRESHOLD_LOW  = 0.93;
unsigned long STEP_THRESHOLD_TIME = 350;
unsigned long STEP_THRESHOLD_TIME_TUNED = STEP_THRESHOLD_TIME;
float STEP_THRESHOLD_HIGH_TUNED = STEP_THRESHOLD_HIGH;

// ================== OBJEKTUMOK ==================
MPU6050 mpu;
WebServer server(80);
WiFiClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

// ================== GLOBÁLIS VÁLTOZÓK ==================
volatile unsigned long hallPulseCount = 0;

// Opto
volatile unsigned long lastOptoTime = 0;
volatile unsigned long optoPulseTime = 0;
volatile bool optoNewPulse = false;

// Sebességek
float speed_kmh = 0.0;
float filteredSpeed_kmh = 0.0;
float beltSpeed_kmh = 0.0;
float slipPercent = 0.0;

// Tilt
float tiltBuffer[MEDIANFILTER_FOR_TILT];
int tiltIndex = 0;
float filteredTilt = 0.0;

// Lépés
float stepBuffer[MEDIANFILTER_FOR_STEP];
int stepIndex = 0;
bool stepDetected = false;
unsigned long lastStepTime = 0;
unsigned long prevStepTime = 0;
float stepRate = 0;

// Egyéb
float temperature = 0.0;
String formattedTime;

// ================== IDŐZÍTÉSEK ==================
unsigned long lastSpeedCalcTime = 0;
unsigned long lastSerialOutputTime = 0;

// ================== INTERRUPTOK ==================
void IRAM_ATTR hallISR() {
  hallPulseCount++;
}

void IRAM_ATTR optoISR() {
  unsigned long now = micros();
  if (now - lastOptoTime < OPTO_DEBOUNCE_US) return;
  optoPulseTime = now - lastOptoTime;
  lastOptoTime = now;
  optoNewPulse = true;
}

// ================== FILE UTILS ==================
size_t getFileSize(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) return 0;
  size_t size = f.size();
  f.close();
  return size;
}

// ================== SERIAL ==================
void serialPortHandling() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("AT ")) {
      float newAlpha = input.substring(6).toFloat();
      if (newAlpha >= 0.0 && newAlpha <= 1.0) {
        ALPHA_TILT = newAlpha;
        Serial.print("New ALPHA_TILT value set: ");
        Serial.println(ALPHA_TILT);
      }
    }

    else if (input.startsWith("SH ")) {
      STEP_THRESHOLD_HIGH = input.substring(3).toFloat();
      Serial.print("New STEP_THRESHOLD_HIGH set: ");
      Serial.println(STEP_THRESHOLD_HIGH);
    }

    else if (input.startsWith("SL ")) {
      STEP_THRESHOLD_LOW = input.substring(3).toFloat();
      Serial.print("New STEP_THRESHOLD_LOW set: ");
      Serial.println(STEP_THRESHOLD_LOW);
    }

    else if (input.startsWith("<!DOCTYPE html>")) {
      File file = LittleFS.open("/index.html", FILE_WRITE);
      if (!file) {
        Serial.println("Hiba az index.html létrehozásakor!");
        return;
      }
      Serial.println("Feltöltés.....");
      file.print(input);

      unsigned long startTime = millis();
      while (millis() - startTime < 1000) {
        while (Serial.available()) {
          char c = Serial.read();
          file.write(c);
          startTime = millis();
        }
      }
      file.close();
      Serial.println("Fájl sikeresen mentve!");
      Serial.println(getFileSize("/index.html"));
    }

    else {
      Serial.println("Invalid command!");
    }
  }
}

// ================== LOGGER ADATFELTÖLTÉS ==================
TreadmillData getNextSample(uint16_t sec) {
  TreadmillData s;

  s.speed_x10 = (uint8_t)constrain(filteredSpeed_kmh * 10.0, 0, 255);

  float inclinePercent = tan(filteredTilt / RAD2DEG) * 100.0;
  inclinePercent = constrain(inclinePercent, 0, 100);
  s.incline_x10 = (uint8_t)(inclinePercent * 10.0);

  s.steps_per_min = (uint8_t)constrain(stepRate, 0, 255);
  s.heart_rate = heartRate;
  s.seconds = sec;

  return s;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallISR, RISING);

  pinMode(OPTO_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OPTO_SENSOR_PIN), optoISR, RISING);

  LittleFS.begin(true);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  timeClient.begin();
  ThingSpeak.begin(client);

  // ===== WEB ROUTE: index.html =====
  server.on("/", HTTP_GET, []() {
    File f = LittleFS.open("/index.html", "r");
    server.streamFile(f, "text/html");
    f.close();
  });

  // ===== WEB ROUTE: /data =====
  server.on("/data", HTTP_GET, []() {
    float inclinePercent = tan(filteredTilt / RAD2DEG) * 100.0;
    inclinePercent = constrain(inclinePercent, 0, 100);

    String json = "{";
    json += "\"speed\":" + String(filteredSpeed_kmh,1) + ",";
    json += "\"belt_speed\":" + String(beltSpeed_kmh,1) + ",";
    json += "\"slip\":" + String(slipPercent,1) + ",";
    json += "\"tilt\":" + String(filteredTilt,1) + ",";
    json += "\"incline\":" + String(inclinePercent,1) + ",";
    json += "\"steps\":" + String(stepRate,1) + ",";
    json += "\"heart\":" + String(heartRate) + ",";
    json += "\"time\":\"" + formattedTime + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });

  // ===== RECORD CONTROL =====
  server.on("/start", HTTP_GET, []() {
    if (!recording) {
      startRecording();
      server.send(200, "text/plain", "Recording started");
    } else {
      server.send(200, "text/plain", "Already recording");
    }
  });

  server.on("/stop", HTTP_GET, []() {
    if (recording) {
      stopRecording();
      server.send(200, "text/plain", "Recording stopped");
    } else {
      server.send(200, "text/plain", "Not recording");
    }
  });

  // ===== FILE HANDLERS =====
  server.on("/list", HTTP_GET, handleListFiles);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/delete", HTTP_DELETE, handleDelete);
  server.on("/deleteAll", HTTP_DELETE, handleDeleteAll);

  server.begin();
  Serial.println("Webserver OK");
}

// ================== LOOP ==================
void loop() {
  server.handleClient();
  timeClient.update();
  formattedTime = timeClient.getFormattedTime();
  unsigned long now = millis();

  serialPortHandling();
  updateHeartRate();
  logTreadmillDataLoop();

  // ===== MPU6050 olvasás (10 ms) =====
  static unsigned long lastMPU = 0;
  if (now - lastMPU >= 10) {
    lastMPU = now;

    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    float axN = ax / 16384.0;
    float azN = -az / 16384.0;

    float tilt = 9.0 - atan2(axN, azN) * RAD2DEG;
    tiltBuffer[tiltIndex] = tilt;
    tiltIndex = (tiltIndex + 1) % MEDIANFILTER_FOR_TILT;

    float tb[MEDIANFILTER_FOR_TILT];
    memcpy(tb, tiltBuffer, sizeof(tb));
    std::sort(tb, tb + MEDIANFILTER_FOR_TILT);
    filteredTilt = ALPHA_TILT * tb[MEDIANFILTER_FOR_TILT/2]
                 + (1-ALPHA_TILT)*filteredTilt;

    bool dead = (now - lastStepTime <= STEP_THRESHOLD_TIME_TUNED);
    if (azN > STEP_THRESHOLD_HIGH_TUNED && !stepDetected && !dead) {
      stepDetected = true;
      lastStepTime = now;
      stepBuffer[stepIndex] = lastStepTime - prevStepTime;
      prevStepTime = lastStepTime;

      float sb[MEDIANFILTER_FOR_STEP];
      memcpy(sb, stepBuffer, sizeof(sb));
      std::sort(sb, sb + MEDIANFILTER_FOR_STEP);
      float med = sb[MEDIANFILTER_FOR_STEP/2];
      stepRate = (med > 200 && med < 2000) ? 60000.0 / med : 0;
      stepIndex = (stepIndex + 1) % MEDIANFILTER_FOR_STEP;
    }
    if (azN < STEP_THRESHOLD_LOW) stepDetected = false;
    if (now - lastStepTime > 3000) stepRate = 0;
  }

  // ===== SEBESSÉG (1 mp) =====
  if (now - lastSpeedCalcTime >= 1000) {
    lastSpeedCalcTime = now;

    speed_kmh = (hallPulseCount * BELT_DISTANCE / 100.0) * 3.6;
    hallPulseCount = 0;
    filteredSpeed_kmh = ALPHA_SPEED * speed_kmh + (1-ALPHA_SPEED)*filteredSpeed_kmh;

    if (optoNewPulse) {
      optoNewPulse = false;
      float v = (BELT_LENGTH_M / (optoPulseTime / 1e6)) * 3.6;
      beltSpeed_kmh = v;
      slipPercent = (filteredSpeed_kmh > 0.1)
          ? (filteredSpeed_kmh - beltSpeed_kmh) / filteredSpeed_kmh * 100.0
          : 0;
    }
  }

  // ===== SERIAL DEBUG =====
  if (now - lastSerialOutputTime >= 1000) {
    lastSerialOutputTime = now;
    Serial.printf("Speed %.1f km/h | Belt %.1f km/h | Slip %.1f %% | Tilt %.1f ° | Steps %.1f | HR %d\n",
                  filteredSpeed_kmh, beltSpeed_kmh, slipPercent, filteredTilt, stepRate, heartRate);
  }
}

// ================== FILE HANDLEREK ==================
void handleListFiles() {
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  String json="[";
  bool first=true;
  while(f){
    if(!first) json+=",";
    json+="\""+String(f.name())+"\"";
    first=false;
    f=root.openNextFile();
  }
  json+="]";
  server.send(200,"application/json",json);
}

void handleDownload() {
  if(!server.hasArg("file")) return;
  File f = LittleFS.open(server.arg("file"),"r");
  server.streamFile(f,"application/octet-stream");
  f.close();
}

void handleDelete() {
  String p = server.arg("file");
  if(p=="/index.html") {
    server.send(403,"text/plain","index protected");
    return;
  }
  LittleFS.remove(p);
  server.send(200,"text/plain","deleted");
}

void handleDeleteAll() {
  File root = LittleFS.open("/");
  File f = root.openNextFile();
  int cnt=0;
  while(f){
    if(String(f.name())!="/index.html"){
      LittleFS.remove(f.name());
      cnt++;
    }
    f=root.openNextFile();
  }
  server.send(200,"text/plain","deleted "+String(cnt));
}
