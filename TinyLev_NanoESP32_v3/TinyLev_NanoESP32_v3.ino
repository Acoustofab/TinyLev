#include <Arduino.h>
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "rom/gpio.h"          // gpio_matrix_out()
#include "soc/gpio_sig_map.h"  // RMT_SIG_OUT0_IDX etc.
#include <NimBLEDevice.h>      // NimBLE-Arduino 2.5.1 — single header covers server/service/characteristic

#define N_DIVS             24
#define N_FRAMES           24
#define N_BUTTONS           6
#define STEP_SIZE           1
#define SYNC_FREQ_HZ     40000UL

#define RMT_CLK_DIV          1
#define RMT_TICK_HZ   80000000UL
#define PERIOD_TICKS       2000UL
#define BUTTON_HOLD_MS       63

// ---------------------------------------------------------------------
// PIN CONFIGURATION
// ---------------------------------------------------------------------
// OUT_PIN_0 / OUT_PIN_1 form one complementary pair (drives AIN1/AIN2 ->
// AO1/AO2 on the TB6612). OUT_PIN_2 / OUT_PIN_3 form the other pair
// (BIN1/BIN2 -> BO1/BO2).
//
// *** WHY THIS VERSION ONLY PROGRAMS 2 RMT CHANNELS INSTEAD OF 4 ***
// Scope captures showed AO1/AO2 not staying in sync on the ESP32,
// while the same points on the original AVR Nano stayed perfectly
// synced. The reason: on AVR, `PORTC = value` updates all 4 output
// bits in one single CPU instruction — bit0 and bit1 physically cannot
// go out of step. On ESP32, each output pin was its own independent
// RMT hardware channel, each started by a separate function call a few
// CPU cycles apart — enough for a small, constant timing skew between
// channels, even though their loop periods were identical.
//
// The animation table always keeps each pair's two bits as exact
// logical opposites (verified: for every value actually used — 0x5,
// 0x6, 0x9, 0xa — bit0/bit1 are always complementary, and so are
// bit2/bit3). So instead of driving both pins of a pair from separate
// RMT channels and hoping they stay aligned, this version drives only
// the FIRST pin of each pair from an RMT channel, and generates the
// SECOND pin as a hardware-inverted mirror of that same signal via the
// ESP32's GPIO matrix (gpio_matrix_out). The two pins are then driven
// by the literal same hardware edge, just electrically inverted —
// they cannot desync, rather than just "closely timed".
//
// This does mean only channel-A-vs-channel-B synchronization (i.e.
// OUT_PIN_0's RMT channel vs OUT_PIN_2's RMT channel) is still software
// -started and could in principle carry a small constant skew of its
// own. If that also turns out to be visible on a scope comparing an
// A-side pin against a B-side pin, that's a separate, deeper fix
// (would need the newer IDF v5 RMT sync-manager APIs, not available in
// the legacy driver this core version ships with) — flag it and we'll
// address it specifically if so.
//
// *** THING TO VERIFY IF THIS DOESN'T COMPILE ***
// RMT_SIG_OUT0_IDX (and RMT_SIG_OUT0_IDX+1 for channel 1) is the
// standard ESP-IDF macro name for "GPIO matrix signal ID for RMT TX
// channel N's output" — this has been consistent across ESP32 variants
// historically, but if your installed core's soc/gpio_sig_map.h uses a
// different name, search that header (inside your Arduino-ESP32 core
// installation folder) for "RMT_SIG" and swap in whatever it defines.
#define OUT_PIN_0   A0   // primary, pair A (bit0) -> AIN1
#define OUT_PIN_1   A1   // mirror,  pair A (bit1, hw-inverted copy of OUT_PIN_0) -> AIN2
#define OUT_PIN_2   A2   // primary, pair B (bit2) -> BIN1
#define OUT_PIN_3   A3   // mirror,  pair B (bit3, hw-inverted copy of OUT_PIN_2) -> BIN2

static const uint8_t buttonPins[N_BUTTONS] = { 2, 3, 4, 5, 6, 7 };

static byte frame = 0;

static byte animation[N_FRAMES][N_DIVS] =
{{0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x6,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x6,0x6,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x6,0x6,0x6,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x6,0x6,0x6,0x6,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x6,0x6,0x6,0x6,0x6,0xa,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x5,0x5,0x5,0x5,0x5,0x5,0x6,0x6,0x6,0x6,0x6,0x6,0xa,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x5,0x5,0x5,0x5,0x5,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0xa,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x5,0x5,0x5,0x5,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0xa,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x5,0x5,0x5,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0xa,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x5,0x5,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0xa,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x5,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0xa},
{0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0xa,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0xa,0xa,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0xa,0xa,0xa,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0xa,0xa,0xa,0xa,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x9,0x9,0x9,0x9,0x9,0x9,0x9,0xa,0xa,0xa,0xa,0xa,0x6,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x5,0x9,0x9,0x9,0x9,0x9,0x9,0xa,0xa,0xa,0xa,0xa,0xa,0x6,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x9,0x9,0x9,0x9,0x9,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0x6,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x9,0x9,0x9,0x9,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0x6,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x9,0x9,0x9,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0x6,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x9,0x9,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0x6,0x6},
{0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x5,0x9,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0xa,0x6}};

static uint16_t stepTicks[N_DIVS];

// Only 2 RMT channels now — one per complementary pair (see comment
// block above OUT_PIN_0). rmtItems[0] carries bit0 (drives OUT_PIN_0
// directly, and OUT_PIN_1 as its hardware-inverted mirror). rmtItems[1]
// carries bit2 (drives OUT_PIN_2 directly, OUT_PIN_3 as its mirror).
static const rmt_channel_t rmtChannels[2] = { RMT_CHANNEL_0, RMT_CHANNEL_1 };
static rmt_item32_t rmtItems[2][N_DIVS / 2];
static const int primaryBit[2] = { 0, 2 };

static void computeStepTicks() {
  const uint32_t base = PERIOD_TICKS / N_DIVS;
  const uint32_t rem = PERIOD_TICKS % N_DIVS;
  uint32_t total = 0;

  for (int d = 0; d < N_DIVS; ++d) {
    stepTicks[d] = (uint16_t)(base + ((uint32_t)d < rem ? 1 : 0));
    total += stepTicks[d];
  }

  if (total != PERIOD_TICKS) {
    Serial.printf("FATAL: RMT timing calculation error: %lu != %lu\n", (unsigned long)total, (unsigned long)PERIOD_TICKS);
    while (true) delay(1000);
  }
}

static void buildRmtItemsForFrame(byte f) {
  for (int g = 0; g < 2; ++g) {
    const int bit = primaryBit[g];
    for (int pair = 0; pair < N_DIVS / 2; ++pair) {
      const int d0 = pair * 2;
      const int d1 = d0 + 1;

      rmtItems[g][pair].level0    = (animation[f][d0] >> bit) & 0x01;
      rmtItems[g][pair].duration0 = stepTicks[d0];
      rmtItems[g][pair].level1    = (animation[f][d1] >> bit) & 0x01;
      rmtItems[g][pair].duration1 = stepTicks[d1];
    }
  }
}

static void setupRmtChannel(int index, int primaryGpio, int mirrorGpio) {
  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX;
  config.channel = rmtChannels[index];
  config.gpio_num = (gpio_num_t)primaryGpio;
  config.clk_div = RMT_CLK_DIV;
  config.mem_block_num = 1;
  config.tx_config.loop_en = true;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_duty_percent = 50;
  config.tx_config.carrier_freq_hz = 38000;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;

  esp_err_t err = rmt_config(&config);
  if (err != ESP_OK) {
    Serial.printf("rmt_config channel %d GPIO %d failed: %s\n", index, primaryGpio, esp_err_to_name(err));
    while (true) delay(1000);
  }

  err = rmt_driver_install(config.channel, 0, 0);
  if (err != ESP_OK) {
    Serial.printf("rmt_driver_install channel %d failed: %s\n", index, esp_err_to_name(err));
    while (true) delay(1000);
  }

  // Mirror this RMT channel's own output signal onto the pair's second
  // pin, inverted, directly at the GPIO matrix. This is the fix: both
  // pins are now driven by the literal same hardware signal edge, just
  // electrically inverted, so they cannot desync from each other.
  gpio_set_direction((gpio_num_t)mirrorGpio, GPIO_MODE_OUTPUT);
  gpio_matrix_out((gpio_num_t)mirrorGpio, RMT_SIG_OUT0_IDX + index, true /*invert*/, false);
}

static void startWaveform() {
  for (int ch = 0; ch < 2; ++ch) {
    esp_err_t err = rmt_write_items(rmtChannels[ch], rmtItems[ch], N_DIVS / 2, false);
    if (err != ESP_OK) {
      Serial.printf("rmt_write_items channel %d failed: %s\n", ch, esp_err_to_name(err));
      while (true) delay(1000);
    }
  }
}

static void stopWaveform() {
  for (int ch = 0; ch < 2; ++ch) {
    esp_err_t err = rmt_tx_stop(rmtChannels[ch]);
    if (err != ESP_OK) {
      Serial.printf("rmt_tx_stop channel %d failed: %s\n", ch, esp_err_to_name(err));
    }
  }
}

static void setFrame(byte newFrame) {
  if (newFrame == frame) return;
  stopWaveform();
  frame = newFrame;
  buildRmtItemsForFrame(frame);
  startWaveform();
}

static bool buttonPressed[N_BUTTONS];
static bool anyButtonPressed = false;
static bool holding = false;
static uint32_t holdStartMs = 0;

#define BLE_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_RX_UUID             "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_TX_UUID             "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static bool bleConnected = false;
static NimBLEServer *pBleServer = nullptr;
static NimBLECharacteristic *pTxCharacteristic = nullptr;
static NimBLECharacteristic *pRxCharacteristic = nullptr;

static void applyFrameDelta(int delta) {
  int newFrame = (int)frame + delta;
  while (newFrame < 0) newFrame += N_FRAMES;
  while (newFrame >= (int)N_FRAMES) newFrame -= N_FRAMES;
  setFrame((byte)newFrame);
}

static void sendBleStatus(const char *label) {
  if (!bleConnected || pTxCharacteristic == nullptr) {
    return;
  }

  char status[32];
  snprintf(status, sizeof(status), "%s:%u\n", label, (unsigned)frame);
  pTxCharacteristic->setValue(status);
  pTxCharacteristic->notify();
}

// Only W (up) / S (down) / PageUp / PageDown / Esc (reset) remain —
// A/D removed as requested. Both ASCII letters and raw HID usage codes
// are accepted, in case the BLE client sends either.
static void handleKeyCommand(uint8_t key) {
  switch (key) {
    case 'W':
    case 'w':
    case 0x4B: // HID PageUp
      applyFrameDelta(+1);
      sendBleStatus("phase");
      break;

    case 'S':
    case 's':
    case 0x4E: // HID PageDown
      applyFrameDelta(-1);
      sendBleStatus("phase");
      break;

    case 0x29: // HID Escape
    case 0x1B: // ASCII ESC
      setFrame(0);
      sendBleStatus("reset");
      break;

    default:
      break;
  }
}

class TinyLevServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    bleConnected = true;
    Serial.println("BLE connected");
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
    bleConnected = false;
    Serial.printf("BLE disconnected, reason=%d\n", reason);
    // Advertising auto-restarts because we called pServer->advertiseOnDisconnect(true)
    // in setupBle() — NimBLE 2.x does NOT do this by default (unlike the old
    // Bluedroid-based library), so that call is required, not optional.
  }
};

class TinyLevCharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &connInfo) override {
    std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }

    for (size_t i = 0; i < value.length(); ++i) {
      uint8_t key = static_cast<uint8_t>(value[i]);
      if (key == '\r' || key == '\n' || key == '\0') {
        continue;
      }
      handleKeyCommand(key);
    }
  }
};

static void setupBle() {
  NimBLEDevice::init("TinyLev");

  pBleServer = NimBLEDevice::createServer();
  pBleServer->setCallbacks(new TinyLevServerCallbacks());
  pBleServer->advertiseOnDisconnect(true); // required in NimBLE 2.x — not the default

  NimBLEService *pService = pBleServer->createService(BLE_SERVICE_UUID);

  pRxCharacteristic = pService->createCharacteristic(
    BLE_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY
  );
  pRxCharacteristic->setCallbacks(new TinyLevCharacteristicCallbacks());

  pTxCharacteristic = pService->createCharacteristic(
    BLE_TX_UUID,
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
  );
  // No manual descriptor needed — NimBLE auto-adds the CCCD for any
  // NOTIFY-capable characteristic (this replaces the old BLE2902 call).

  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  // Scan response is OFF by default in NimBLE 2.x (changed from 1.x).
  // Without it, the device name + our 128-bit service UUID together
  // exceed the 31-byte advertising packet limit, which makes NimBLE
  // fail to start advertising at all (not just "no name shown" — no
  // connection possible). Enabling scan response gives the name its
  // own separate packet, leaving the main packet just flags + UUID.
  pAdvertising->enableScanResponse(true);
  pAdvertising->setName("TinyLev");
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE UART ready: TinyLev");
}

static void pollButtons() {
  anyButtonPressed = false;

  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
    buttonPressed[i] = (digitalRead(buttonPins[i]) == LOW);
    if (buttonPressed[i]) anyButtonPressed = true;
  }

  const uint32_t now = millis();
  if (!anyButtonPressed) {
    holding = false;
    return;
  }

  if (!holding) {
    holding = true;
    holdStartMs = now;
    return;
  }

  if ((uint32_t)(now - holdStartMs) < BUTTON_HOLD_MS) return;

  holdStartMs = now;

  if (buttonPressed[0]) {
    const byte newFrame = (frame < STEP_SIZE) ? (N_FRAMES - 1) : (frame - STEP_SIZE);
    setFrame(newFrame);
  } else if (buttonPressed[1]) {
    const byte newFrame = (frame >= N_FRAMES - STEP_SIZE) ? 0 : (frame + STEP_SIZE);
    setFrame(newFrame);
  } else if (buttonPressed[2]) {
    setFrame(0);
  }
}

static void printTiming(const int gpio[4]) {
  Serial.println();
  Serial.println("----------------------------------------");
  Serial.println("TinyLev Nano ESP32 - Legacy RMT (hw-synced pairs)");
  Serial.println("----------------------------------------");
  Serial.printf("RMT clock:       %lu Hz\n", (unsigned long)RMT_TICK_HZ);
  Serial.printf("Target frequency: %lu Hz\n", (unsigned long)SYNC_FREQ_HZ);
  Serial.printf("Period:          %lu ticks = %.3f us\n", (unsigned long)PERIOD_TICKS, (double)PERIOD_TICKS * 1000000.0 / (double)RMT_TICK_HZ);
  Serial.printf("Actual frequency: %.3f Hz\n", (double)RMT_TICK_HZ / (double)PERIOD_TICKS);
  Serial.println();
  Serial.println("Outputs (Arduino pin -> resolved GPIO):");
  Serial.printf("  A0 -> GPIO%d = pair A primary (RMT ch0)\n", gpio[0]);
  Serial.printf("  A1 -> GPIO%d = pair A mirror, hw-inverted\n", gpio[1]);
  Serial.printf("  A2 -> GPIO%d = pair B primary (RMT ch1)\n", gpio[2]);
  Serial.printf("  A3 -> GPIO%d = pair B mirror, hw-inverted\n", gpio[3]);
  Serial.println();
  Serial.println("Buttons: D2=previous  D3=next  D4=reset  D5-D7=unused");
  Serial.println("BLE keys: W/PageUp=next  S/PageDown=previous  Esc=reset");
  Serial.println("----------------------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  computeStepTicks();

  int gpio[4] = {
    digitalPinToGPIONumber(OUT_PIN_0),
    digitalPinToGPIONumber(OUT_PIN_1),
    digitalPinToGPIONumber(OUT_PIN_2),
    digitalPinToGPIONumber(OUT_PIN_3)
  };

  setupRmtChannel(0, gpio[0], gpio[1]); // pair A: primary + inverted mirror
  setupRmtChannel(1, gpio[2], gpio[3]); // pair B: primary + inverted mirror

  buildRmtItemsForFrame(frame);
  startWaveform();
  setupBle();
  printTiming(gpio);
}

void loop() {
  pollButtons();
  delay(1);
}
