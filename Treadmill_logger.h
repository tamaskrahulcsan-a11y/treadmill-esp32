#ifndef TREADMILL_LOGGER_H
#define TREADMILL_LOGGER_H

#include <LittleFS.h>
#include <time.h>
#include "BLE_HeartRate.h"

/*******************************************************
 * FUTÓPAD ADATRÖGZÍTŐ – ÖSSZEHANGOLT VERZIÓ AZ .INO-VAL
 *
 * - Nem blokkol (delay eltávolítva)
 * - A start/stop vezérlést az .ino kezeli
 * - Valós adatokat rögzít (sebesség, dőlés, lépésszám, pulzus)
 * - 256 bájtos blokkokban ír
 * - Idle stop logika megmarad
 *******************************************************/


struct TreadmillData {
  uint8_t speed_x10;
  uint8_t incline_x10;
  uint8_t steps_per_min;
  uint8_t heart_rate;
  uint16_t seconds;
};

// A fő .ino fájlban definiált függvény deklarációja:
TreadmillData getNextSample(uint16_t sec);

const int BLOCK_SIZE = 256;
const int RECORD_SIZE = sizeof(TreadmillData);
const int RECORDS_PER_BLOCK = BLOCK_SIZE / RECORD_SIZE;

TreadmillData buffer[RECORDS_PER_BLOCK];
int bufferIndex = 0;

File dataFile;
String filename = "";
bool recording = false;
uint16_t currentSecond = 0;
uint16_t idleCounter = 0;

// =====================================================
// FÁJLNÉV GENERÁLÁS – marad a régi formátum
// =====================================================
String getFormattedFilename() {
  time_t now = time(nullptr);
  struct tm* tm_info = localtime(&now);
  char b[32];
  strftime(b, sizeof(b), "/%Y%m%d_%H%M%S.txt", tm_info);
  return String(b);
}

// =====================================================
// RÉGI HELYKITÖLTŐ FUNKCIÓ — ÁTNEVEZVE, HOGY NE ÜTKÖZZÖN
// =====================================================
TreadmillData getNextSample_placeholder(uint16_t sec) {
  TreadmillData s;
  s.speed_x10 = 0;
  s.incline_x10 = 0;
  s.steps_per_min = 0;
  s.heart_rate = heartRate;
  s.seconds = sec;
  return s;
}

// =====================================================
// FELVÉTEL INDÍTÁSA – az .ino hívja meg
// =====================================================
void startRecording() {
  filename = getFormattedFilename();
  dataFile = LittleFS.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Nem sikerült megnyitni a fájlt!");
    return;
  }
  Serial.println("Felvétel indítva: " + filename);
  recording = true;
  bufferIndex = 0;
  idleCounter = 0;
  currentSecond = 0;
}

// =====================================================
// FELVÉTEL LEÁLLÍTÁSA – az .ino hívja meg
// =====================================================
void stopRecording() {
  if (bufferIndex > 0 && dataFile) {
    dataFile.write((uint8_t*)buffer, bufferIndex * RECORD_SIZE);
  }
  if (dataFile) dataFile.close();
  recording = false;
  Serial.println("Felvétel vége, fájl lezárva.");
}

// =====================================================
// LOGOLÓ CIKLUS – NEM BLOKKOL, AZ .INO IDŐZÍTI
// =====================================================
void logTreadmillDataLoop() {

  if (!recording) return;

  // Az .ino 1 másodpercenként hívja
  TreadmillData sample = getNextSample(currentSecond++);

  // Pulzus mindig friss
  sample.heart_rate = heartRate;

  buffer[bufferIndex++] = sample;

  // 256 bájtos blokk megtelt → mentés
  if (bufferIndex >= RECORDS_PER_BLOCK) {
    dataFile.write((uint8_t*)buffer, bufferIndex * RECORD_SIZE);
    dataFile.flush();
    bufferIndex = 0;
    Serial.println("256 bájt blokk mentve.");
  }

  // Idle stop logika
  if (sample.speed_x10 == 0 && sample.steps_per_min == 0) {
    idleCounter++;
  } else {
    idleCounter = 0;
  }

  if (idleCounter >= 60) {
    stopRecording();
  }
}

#endif
