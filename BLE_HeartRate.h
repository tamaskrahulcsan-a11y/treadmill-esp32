#ifndef BLE_HEARTRATE_H
#define BLE_HEARTRATE_H

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

static BLEUUID hrServiceUUID((uint16_t)0x180D);
static BLEUUID hrCharUUID((uint16_t)0x2A37);

BLEAddress* pHRDeviceAddress = nullptr;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteHRChar = nullptr;

bool doConnect = false;
bool hrConnected = false;
uint8_t heartRate = 0;

unsigned long lastBLEScanTime = 0;
bool scanRunning = false;
unsigned long lastConnectAttempt = 0;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(hrServiceUUID)) {
      Serial.print("Pulzusöv megtalálva: ");
      Serial.println(advertisedDevice.toString().c_str());
      if (pHRDeviceAddress) delete pHRDeviceAddress;
      pHRDeviceAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
    }
  }
};

inline void startBLEScan() {
  static bool inited = false;
  if (!inited) {
    BLEDevice::init("");
    inited = true;
  }

  BLEScan* pBLEScan = BLEDevice::getScan();
  //pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  static MyAdvertisedDeviceCallbacks advCallbacks;
  pBLEScan->setAdvertisedDeviceCallbacks(&advCallbacks);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  scanRunning = true;
  lastBLEScanTime = millis();
}

inline void stopBLEScan() {
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->stop();
  scanRunning = false;
}

inline bool connectToHRM() {
  if (!pHRDeviceAddress) return false;
  if (millis() - lastConnectAttempt < 3000) return false;
  lastConnectAttempt = millis();

  Serial.println("Kapcsolódás a pulzusövhöz...");

  if (pClient) {
    delete pClient;
    pClient = nullptr;
  }

  pClient = BLEDevice::createClient();
  if (!pClient->connect(*pHRDeviceAddress)) {
    Serial.println("Sikertelen csatlakozás.");
    return false;
  }

  BLERemoteService* pRemoteService = pClient->getService(hrServiceUUID);
  if (!pRemoteService) {
    Serial.println("HRM szolgáltatás nem található.");
    pClient->disconnect();
    return false;
  }

  pRemoteHRChar = pRemoteService->getCharacteristic(hrCharUUID);
  if (!pRemoteHRChar) {
    Serial.println("HRM karakterisztika nem található.");
    pClient->disconnect();
    return false;
  }

  hrConnected = true;
  Serial.println("Kapcsolódva a pulzusövhöz!");

  stopBLEScan();
  return true;
}

inline void disconnectHRM() {
  if (pClient) {
    pClient->disconnect();
    delete pClient;
    pClient = nullptr;
  }
  hrConnected = false;
  pRemoteHRChar = nullptr;
}

inline void updateHeartRate() {
  if (!hrConnected) {
    if (!scanRunning || (millis() - lastBLEScanTime > 6000)) {
      startBLEScan();
    }
    if (doConnect) {
      doConnect = false;
      if (connectToHRM()) {
        Serial.println("Connected after found device.");
      }
    }
    return;
  }

  if (pClient && !pClient->isConnected()) {
    Serial.println("HRM connection lost, disconnecting and restarting scan...");
    disconnectHRM();
    startBLEScan();
    return;
  }

  if (pRemoteHRChar && pRemoteHRChar->canRead()) {
    String raw = pRemoteHRChar->readValue();
    int len = raw.length();
    if (len > 1) {
      const uint8_t* data = (const uint8_t*)raw.c_str();
      bool hr16 = (data[0] & 0x01);
      if (!hr16) {
        heartRate = data[1];
      } else {
        if (len >= 3) heartRate = (uint16_t)((uint8_t)data[1] | ((uint8_t)data[2] << 8));
      }
    }
  }
}

#endif
