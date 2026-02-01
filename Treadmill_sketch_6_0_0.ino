/*******************************************************
 * ESP32 FUTÓPAD ADATGYŰJTŐ ÉS WEBSZERVER  6_0_0
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
#include <WiFi.h>  // Ha ESP32-t használsz, kell a WiFi könyvtár
#include <WebServer.h>
#include <LittleFS.h>
#include <NTPClient.h> // Net idő
#include <WiFiUdp.h>

#include "BLE_HeartRate.h"
#include "Treadmill_logger.h"

#include "netdef.h" //Az alábbi 4 darab hálózati azonosítók definíciós file-ja.
//#define WIFI_SSID 
//#define WIFI_PASSWORD 
//#define THINGSPEAK_API_KEY   
//#define YOUR_CHANNEL_ID 

#define DEBUG 1  //

// ================== HARDVER KONFIG ==================
#define HALL_SENSOR_PIN 15  //szalagmozgató orsón levő szenzor 1 impulzus fordulatonként. Orsó átmérő 49mm
#define BELT_DISTANCE 15.25 //két impulzus közti szalagmozgás cm
#define HALL_DEBOUNCE_US 15000  //15ms  not implemented yet

#define OPTO_SENSOR_PIN 27 //szalag 1 körbefordulásának detektálása
#define BELT_LENGTH_M 2.85
#define OPTO_DEBOUNCE_US 300000  //300ms

#define RAD2DEG 57.29578

// ================== SZŰRÉSI PARAMÉTEREK ==================
#define MEDIANFILTER_FOR_TILT 41     // Exponenciális szűrő súlyozási tényezője, incline detektálás
#define MEDIANFILTER_FOR_STEP 11

float ALPHA_TILT  = 0.01;  // Exponenciális szűrő súlyozási tényezője, incline detektálás
float ALPHA_SPEED = 0.5;   // Exponenciális szűrő súlyozási tényezője, sebesség detektálás

// Lépésdetektálás paraméterek
float STEP_THRESHOLD_HIGH = 1.13;   // Lépésérzékelési küszöb felső érték (lágy lépéshez)
float STEP_THRESHOLD_LOW  = 0.93;  // Lépésérzékelési küszöb alsó érték
unsigned long STEP_THRESHOLD_TIME = 350;   // Lépések közti idő amíg nem engedünk új lépést detektálni msec
unsigned long STEP_THRESHOLD_TIME_TUNED = STEP_THRESHOLD_TIME;  //Sebesség függvényében változhat
float STEP_THRESHOLD_HIGH_TUNED = STEP_THRESHOLD_HIGH;   //Sebesség függvényében változhat

// ================== OBJEKTUMOK ==================
MPU6050 mpu;
WebServer server(80);
WiFiClient client;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

// ================== GLOBÁLIS VÁLTOZÓK ==================
volatile unsigned long hallPulseCount = 0;   //Orsó sebesség 
volatile unsigned long hallPulseCount2 = 0;  //Orsó sebesség relatív a szalagsebességhez detektáláás

// Opto
volatile unsigned long lastOptoTime = 0;
volatile unsigned long optoPulseTime = 0;
volatile unsigned long elapsedHallPulse = 0; //Szalagcsúszás detektálás
volatile bool optoNewPulse = false;

// Sebességek
float speed_kmh = 0.0;
float filteredSpeed_kmh = 0.0;
float beltSpeed_kmh = 0.0;
float slipPercent = 0.0;

float speedHistory[2] = { 0.0, 0.0 }; // Sebesség értékek tárolása az utolsó 2 méréshez
int speedIndex = 0;

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
float temperature = 0.0;    // Hőmérséklet változó
String formattedTime;
char buf[16]; //for time
int debugLevel;
bool heartRateBLEEnabled = false;

// ================== IDŐZÍTÉSEK ==================
unsigned long lastSpeedCalcTime = 0;
unsigned long lastSerialOutputTime = 0;

// ================== INTERRUPTOK ==================
void IRAM_ATTR hallISR() {  // szalagmozgató orsó fordulat detektálás
  hallPulseCount = hallPulseCount + 1;   // sebesség érzékelés
  hallPulseCount2 = hallPulseCount2 +1;  // szalagcsúszás érzékeléd
}

void IRAM_ATTR optoISR() {  // szalagfordulat detektálás
  unsigned long now = micros();
  if (now - lastOptoTime < OPTO_DEBOUNCE_US) return;
  optoPulseTime = now - lastOptoTime;
  lastOptoTime = now;
  optoNewPulse = true;  
  elapsedHallPulse = hallPulseCount2;
  hallPulseCount2 = 0;
}


// ================== UTILs ==================
/* Buffer-alapú változat (javasolt beágyazott rendszeren) */
void formatTimeFromMsBuf(unsigned long ms, char *outBuf, size_t outBufSize) {
  if (!outBuf || outBufSize < 12) { // legalább "hh:mm:ss:cc\0"
    if (outBuf && outBufSize > 0) outBuf[0] = '\0';
    return;
  }
  unsigned long totalCs = ms / 10UL;
  unsigned int cs = totalCs % 100UL;
  unsigned long totalSec = ms / 1000UL;
  unsigned int sec = totalSec % 60UL;
  unsigned long totalMin = totalSec / 60UL;
  unsigned int min = totalMin % 60UL;
  unsigned long hours = totalMin / 60UL;
  snprintf(outBuf, outBufSize, "%02lu:%02u:%02u:%02u", hours, min, sec, cs);
}


// ================== FILE UTILS ==================
size_t getFileSize(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.println("Hiba! Nem sikerült megnyitni a fájlt.");
    return 0;
  }
  size_t size = f.size();
  f.close();
  return size;
}

void printFileSystemSpace() {
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;

  Serial.printf("LittleFS teljes méret: %u B\n", totalBytes);
  Serial.printf("Felhasznált: %u B\n", usedBytes);
  Serial.printf("Szabad hely: %u B\n", freeBytes);
}


// ================== SERIAL ==================
void serialPortHandling() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("AT ")) {
      float newAlpha = input.substring(3).toFloat();
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

    else if (input.startsWith("DEB ")) {
      debugLevel = input.substring(4).toInt();
      Serial.print("New DEBUG level [0-3]: ");
      Serial.println(debugLevel);
    }

    else if (input.startsWith("HR ")) {
      heartRateBLEEnabled = input.substring(3) == "1";
      Serial.print("HR BLE enabled [0-1]: ");
      Serial.println(heartRateBLEEnabled?1:0);
    }

    else if (input.startsWith("<!DOCTYPE html>")) {  //soros porton keresztül index.html (webszerver) feltöltése
      File file = LittleFS.open("/index.html", FILE_WRITE);
      if (!file) {
        Serial.println("Hiba az index.html létrehozásakor!");
        return;
      }
      Serial.println("Feltöltés.....");
      file.print(input); // Az első sor mentése

      unsigned long startTime = millis();  //  Folytatjuk az adatfogadást, amíg Serial-on még érkezik adat
      while (millis() - startTime < 1000) {  // 1 másodperc timeout
        while (Serial.available()) {
          char c = Serial.read();
          file.write(c);
          startTime = millis();  // Reset timeout, ha adat érkezik
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


// ================== FILE HANDLEREK ==================
void handleListFiles() {
  File root = LittleFS.open("/");
  if (!root) {
    server.send(500, "text/plain", "FS open error");
    return;
  }
  File f = root.openNextFile();
  String json = "[";
  bool first = true;
  while (f) {
    if (!first) json += ",";
    json += "\"" + String(f.name()) + "\"";
    first = false;
    f.close();                    // <-- zárjuk le a fájlt, mielőtt a következőt nyitjuk
    f = root.openNextFile();
  }
  json += "]";
  root.close();                   // <-- zárjuk le a root-ot
  server.send(200, "application/json", json);
}

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "missing file param");
    return;
  }
  String path = server.arg("file");
  if (!LittleFS.exists(path)) {
    server.send(404, "text/plain", "not found");
    return;
  }
  File f = LittleFS.open(path, "r");
  if (!f) {
    server.send(500, "text/plain", "unable to open file");
    return;
  }
  server.streamFile(f, "application/octet-stream");
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
  if (!root) {
    server.send(500, "text/plain", "FS open error");
    return;
  }
  File f = root.openNextFile();
  int cnt = 0;
  while (f) {
    String name = String(f.name());
    if (name != "/index.html") {
      LittleFS.remove(f.name());
      cnt++;
    }
    f.close();
    f = root.openNextFile();
  }
  root.close();
  server.send(200, "text/plain", "deleted " + String(cnt));
}


// ================== SETUP ==================
void setup() {
  debugLevel = DEBUG;
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallISR, RISING);

  pinMode(OPTO_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OPTO_SENSOR_PIN), optoISR, RISING);

   if (!LittleFS.begin(true)) {  //ESP32 fájlrendszer inicializálása, a WEBSZERVER HTML adatait tartalmazza
    Serial.println("LittleFS mount failed!");
  }
  printFileSystemSpace();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

    // Kiírjuk az IP címet a soros portra
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());  // Kiírja az IP címet

  timeClient.begin();
  //timeClient.setTimeOffset(3600);
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
  static unsigned long lastThingSpeakTime = 0;
  unsigned long timeDeviation;

  server.handleClient();
  timeClient.update();
  formattedTime = timeClient.getFormattedTime();
  unsigned long now = millis();

  serialPortHandling();
  if (heartRateBLEEnabled){
    updateHeartRate();
  }
  logTreadmillDataLoop();

  // ===== MPU6050 olvasás (10 ms) =====
  static unsigned long lastMPU = 0;
  if (now - lastMPU >= 10) {      // MPU6050 adatok kiolvasása 10 ms-onként
    lastMPU = now;

    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float axN = ax / 16384.0;   // Normalizált
    float azN = -az / 16384.0;  // Előjel megfordítása a szenzor fejtetőn való beszerelése miatt

    //Futópad dőlésszög mérésnek a zajszűrése. Utolsó MEDIANFILTER_FOR_TILT adatból medianTiltAngle számítása és ez lesz az exponenciális szűrőnek a bemenő paramétere
    float tilt = 9.0 - atan2(axN, azN) * RAD2DEG;  //- Dőlésszög, a futópad emelkedési szöge, incline
    tiltBuffer[tiltIndex] = tilt; 
    tiltIndex = (tiltIndex + 1) % MEDIANFILTER_FOR_TILT;
    float tb[MEDIANFILTER_FOR_TILT];
    memcpy(tb, tiltBuffer, sizeof(tb));
    std::sort(tb, tb + MEDIANFILTER_FOR_TILT);
    filteredTilt = ALPHA_TILT * tb[MEDIANFILTER_FOR_TILT/2] + (1-ALPHA_TILT)*filteredTilt;// Exponenciális szűrés alkalmazása a futópad dőlésszögére

    // Lépésszámlálás az Z tengely gyorsulásváltozása alapján    _TUNED paraméterek a szalag sebesség függvényében változnak
    bool dead = (now - lastStepTime <= STEP_THRESHOLD_TIME_TUNED);
    if (azN > STEP_THRESHOLD_HIGH_TUNED && !stepDetected && !dead) {
      stepDetected = true;
      lastStepTime = now;
      stepBuffer[stepIndex] = lastStepTime - prevStepTime;
      prevStepTime = lastStepTime;

      float sb[MEDIANFILTER_FOR_STEP];
      memcpy(sb, stepBuffer, sizeof(sb));
      std::sort(sb, sb + MEDIANFILTER_FOR_STEP);
      float medianStepTime = sb[MEDIANFILTER_FOR_STEP/2];
      stepRate = (medianStepTime > 200 && medianStepTime < 2000) ? 60000.0 / medianStepTime : 0;
      stepIndex = (stepIndex + 1) % MEDIANFILTER_FOR_STEP;
      if (debugLevel == 3) {
        Serial.print(stepIndex); Serial.print("  ");
        Serial.print(stepBuffer[stepIndex]); Serial.print("   ");
        Serial.print(lastStepTime); Serial.print("   ");
        Serial.print(lastStepTime - prevStepTime); Serial.print("   "); 
        Serial.print(medianStepTime); Serial.print("  ");
        Serial.print(stepRate); Serial.println("  ");
      }
    }
    if (azN < STEP_THRESHOLD_LOW) stepDetected = false; // Ha a gyorsulás visszaesik, engedélyez új lépést
    if (now - lastStepTime > 3000) stepRate = 0;  // Ha 3 másodpercig nincs lépés, kinullázza a stepRate-et

    if (debugLevel == 2) {  //grafikon
      Serial.print(azN * 1000);      Serial.print(", ");
      Serial.print(stepDetected ? 1000 : -1000);      Serial.print(", ");
      Serial.print(dead ? 500 : -500); Serial.print(", "); //stepDetectionDeadTimeActive
      Serial.print(-1500); Serial.print(", ");
      Serial.print(1500);  Serial.print(", ");
      Serial.println();
    }
  }

  // ===== SEBESSÉG (1 mp) =====
  timeDeviation = now - lastSpeedCalcTime;
  if (timeDeviation >= 1000) {
    lastSpeedCalcTime = now;

    speed_kmh = (hallPulseCount * BELT_DISTANCE / 100.0) * 3.6;  // cm -> m, m/s -> km/h
    hallPulseCount = 0;
    speed_kmh = speed_kmh / (timeDeviation / 1000.0);  //ha jelentősen több az eltelt idő (pld. wifi hangs) ezzel visszakorrigál...

    speedHistory[speedIndex] = speed_kmh;  // Sebességérték tárolása, csak az utolsó kettő
    speedIndex = (speedIndex + 1) % 2;  // Index körbeforgása (0, 1)
 
    float speedSum = speedHistory[0] + speedHistory[1]; // Az utolsó 2 sebesség átlagolása és szűrése
    speed_kmh = speedSum / 2.0;
    filteredSpeed_kmh = ALPHA_SPEED * speed_kmh + (1-ALPHA_SPEED)*filteredSpeed_kmh;

    if (optoNewPulse) {
      optoNewPulse = false;
      float v = (BELT_LENGTH_M / (optoPulseTime / 1e6)) * 3.6; 
      beltSpeed_kmh = v;
      slipPercent = (filteredSpeed_kmh > 0.1) ? (filteredSpeed_kmh - beltSpeed_kmh) / filteredSpeed_kmh * 100.0  : 0;
    }

    if (filteredSpeed_kmh < 0.2) {
      beltSpeed_kmh = 0;
      slipPercent = 0;
    }

    // Lépésszámláló érzékenységének finomhangolása a sebesség függvényében
    STEP_THRESHOLD_TIME_TUNED = STEP_THRESHOLD_TIME - (filteredSpeed_kmh * 12.0);
    if (filteredSpeed_kmh > 5) { 
      STEP_THRESHOLD_HIGH_TUNED = STEP_THRESHOLD_HIGH + 0.02; 
    }

    temperature = mpu.getTemperature() / 340.0 + 36.53;  // Az MPU6050 hőmérséklet korrekciója
  }

  if (elapsedHallPulse > 0){
    Serial.printf("%s | Speed %.1f km/h | Belt %.1f km/h | Pulse %lu \n", formattedTime.c_str(), filteredSpeed_kmh, beltSpeed_kmh, elapsedHallPulse);
    elapsedHallPulse = 0;
  }


  // ===== SERIAL DEBUG =====
  if (now - lastSerialOutputTime >= 1000) {
    lastSerialOutputTime = now;
    if (debugLevel == 1) {
      Serial.printf("Speed %.1f km/h | Belt %.1f km/h | Slip %.1f %% | Tilt %.1f ° | Steps %.1f | HR %d | Temp %.1f °C \n",
                  filteredSpeed_kmh, beltSpeed_kmh, slipPercent, filteredTilt, stepRate, heartRate, temperature);
    }
  }
//Thingspeak kikapcsolva ideiglenesen
  // ThingSpeak küldés 20 másodpercenként
  // if (now - lastThingSpeakTime >= 20000) {
  //   lastThingSpeakTime = now;
  //   if ((speedHistory[0] + speedHistory[1]) > 0.0) {  //adatküldés csak akkor, ha az utolsó két másodpercben nem nulla
  //     // Adatok feltöltése a ThingSpeak-re
  //     ThingSpeak.setField(1, filteredSpeed_kmh);  // Sebesség
  //     ThingSpeak.setField(2, filteredTilt);       // Dőlésszög
  //     ThingSpeak.setField(3, stepRate);           // Lépésszám (percre vetítve)
  //     ThingSpeak.setField(4, temperature);        // Hőmérséklet
  //     ThingSpeak.writeFields(YOUR_CHANNEL_ID, THINGSPEAK_API_KEY); // Feltöltés a ThingSpeak szerverre
  //   }
  // }
}



