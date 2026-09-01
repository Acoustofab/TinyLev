// Minimal BLE-only test sketch — no RMT, no buttons, nothing else.
// Purpose: isolate whether the connect/disconnect cycling happens even
// in the simplest possible NimBLE server, completely independent of
// the TinyLev code. If this ALSO cycles, the cause is external
// (RF environment, client device, or this board's BLE hardware) —
// not anything in the TinyLev sketch.

#include <NimBLEDevice.h>

#define TEST_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define TEST_CHAR_UUID     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

class TestServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    Serial.printf("[%lu ms] Connected\n", millis());
  }
  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
    Serial.printf("[%lu ms] Disconnected, reason=%d\n", millis(), reason);
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Minimal BLE test starting...");

  NimBLEDevice::init("TinyLevTest");

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new TestServerCallbacks());
  pServer->advertiseOnDisconnect(true);

  NimBLEService *pService = pServer->createService(TEST_SERVICE_UUID);
  NimBLECharacteristic *pChar = pService->createCharacteristic(
    TEST_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pChar->setValue("hello");
  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  // See note in main TinyLev sketch: scan response must be enabled
  // explicitly in NimBLE 2.x, or name + 128-bit UUID together exceed
  // the 31-byte advertising packet limit and advertising fails outright.
  pAdvertising->enableScanResponse(true);
  pAdvertising->setName("TinyLevTest");
  pAdvertising->addServiceUUID(TEST_SERVICE_UUID);
  pAdvertising->start();

  Serial.println("Advertising as 'TinyLevTest' — connect and watch Serial for connect/disconnect timestamps.");
}

void loop() {
  delay(1000);
}
