/*

  TinyLev firmware — Arduino Nano ESP32 port (RMT hardware waveform)
  --------------------------------------------------------------------
  Rebuilt from the original AVR/ATmega328 (Arduino Nano) sketch, which
  used direct PORTC/PIND register access, a Timer1-generated 40kHz
  sync signal looped back through a jumper wire (pin 10 -> pin 11),
  and hand-tuned NOP delay loops. None of that exists on ESP32, and
  earlier attempts at a software busy-wait port were vulnerable to
  timing jitter from ESP32's underlying scheduler (FreeRTOS), which
  showed up as an audible artifact on top of the ultrasonic carrier.

  This version instead uses the ESP32's RMT peripheral — hardware
  originally built for generating IR remote-control pulse trains, and
  equally good at "output exactly this repeating pattern of pin
  transitions, forever, with zero CPU involvement." One RMT channel
  drives each of the 4 output pins directly, in hardware, looping
  indefinitely. Once programmed, the CPU is completely uninvolved in
  producing the waveform: no busy-wait, no interrupt sensitivity, no
  jitter, and no jumper wire needed (the ESP32 just tracks its own
  40kHz timing internally rather than looping a signal back to itself
  through two pins the way the original did).

  TIMING, matched carefully to the original:
  The original's 23 explicit NOP-delays (1x WAIT_LIT=9 nops, 7x
  WAIT_MID=13 nops, 15x WAIT_LOT=14 nops) sum to exactly 310 "weight
  units". The 24th gap — step 23 wrapping back around to step 0 — was
  never explicitly timed in the original; it was simply whatever time
  was left over within the 25us period, governed by the external
  40kHz hardware timer. This version reproduces that faithfully:
  steps 0-22 get durations scaled from the same 310-unit total, and
  step 23's duration is calculated as the exact remainder needed to
  complete one full 25us (40kHz) period — not an invented extra
  category, just like the original.

  *** IF FIRMWARE LOOKS CORRECT BUT LEVITATION STILL DOESN'T WORK ***
  The Arduino Nano ESP32 outputs 3.3V logic; the original Arduino Nano
  output 5V. If your MOSFET/gate-driver board needs a 5V signal to
  fully switch on, you can get a technically-correct, audible,
  correctly phase-shifting signal that is simply too weak to drive the
  transducers with enough acoustic power to levitate anything. Worth
  checking your driver board's input voltage spec / switching
  threshold with a multimeter or scope once you've confirmed the
  firmware itself is behaving as expected.

  *** THINGS TO VERIFY ON YOUR HARDWARE / BUILD ***
  - RMT_CHANNEL_0..3 are used for the 4 output pins. Channel
    enumeration/count for TX use can vary between ESP32 variants and
    ESP-IDF/Arduino-ESP32 core versions — if the compiler complains
    about channel count or capability, check driver/rmt.h for your
    installed core version and adjust rmtChannels[] accordingly.
  - This uses the *legacy* driver/rmt.h API (matches Arduino-ESP32
    core 2.0.18). If you upgrade to core 3.x later, the newer
    driver/rmt_tx.h API is different and this will need porting again.
*/

#include "driver/rmt.h"

#define N_DIVS 24
#define N_FRAMES 24
#define N_BUTTONS 6
#define STEP_SIZE 1
#define SYNC_FREQ_HZ 40000
#define BUTTON_HOLD_MS 62   // ~= original BUTTON_SENS (2500 loop iterations @ 40kHz)

// Uncomment to print frame-change debug info to Serial.
#define BUTTON_DEBUG

// ---------------------------------------------------------------------
// PIN CONFIGURATION — adjust these to match your ESP32 board's wiring.
// The four OUT pins carry the same 4-bit nibble that used to go to
// PORTC bits 0-3 (i.e. bit0..bit3 of animation[frame][d]).
// ---------------------------------------------------------------------
#define OUT_PIN_0   A0   // was PC0
#define OUT_PIN_1   A1   // was PC1
#define OUT_PIN_2   A2   // was PC2
#define OUT_PIN_3   A3   // was PC3

// Button pins (was PIND bits 2-7 / Arduino D2-D7)
static const uint8_t buttonPins[N_BUTTONS] = {2, 3, 4, 5, 6, 7};

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

// ---------------------------------------------------------------------
// Timing — relative proportions taken directly from the original AVR
// NOP counts (WAIT_LIT=9, WAIT_MID=13, WAIT_LOT=14 nops).
// ---------------------------------------------------------------------
#define WEIGHT_LIT 9
#define WEIGHT_MID 13
#define WEIGHT_LOT 14

#define RMT_CLK_DIV 1
#define RMT_TICK_HZ (80000000UL / RMT_CLK_DIV)                          // 80MHz -> 12.5ns/tick
#define PERIOD_TICKS ((uint32_t)((uint64_t)RMT_TICK_HZ / SYNC_FREQ_HZ))  // ticks per 40kHz period (2000 exactly)

static uint32_t stepDuration[N_DIVS]; // RMT ticks each of the 24 steps holds its level for

// Computes stepDuration[] so steps 0-22 keep the original's exact
// relative proportions (1x LIT, 7x MID, 15x LOT — 310 weight units
// total, matching the original NOP counts exactly), and step 23 (the
// wrap back to step 0) gets whatever's left over to complete exactly
// one 40kHz period — the same way the original left that gap to the
// external hardware timer rather than assigning it an explicit delay.
static void computeStepDurations() {
  const uint32_t totalWeight = WEIGHT_LIT + 7 * WEIGHT_MID + 15 * WEIGHT_LOT; // = 310
  const uint32_t unit = PERIOD_TICKS / totalWeight;

  stepDuration[0] = unit * WEIGHT_LIT;                                   // after step0
  for (int d = 1; d <= 7; ++d)  stepDuration[d] = unit * WEIGHT_MID;      // after steps1-7 (7x)
  for (int d = 8; d <= 22; ++d) stepDuration[d] = unit * WEIGHT_LOT;      // after steps8-22 (15x)

  uint32_t used = 0;
  for (int d = 0; d < N_DIVS - 1; ++d) used += stepDuration[d];
  stepDuration[N_DIVS - 1] = PERIOD_TICKS - used; // step23: true remainder, not an invented category
}

// ---------------------------------------------------------------------
// RMT setup — one channel per output pin, each looping its own 12-item
// (24-step) instruction list forever, entirely in hardware.
// ---------------------------------------------------------------------
static const rmt_channel_t rmtChannels[4] = {
  RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3
};

static rmt_item32_t rmtItems[4][N_DIVS / 2]; // 2 steps pack into one rmt_item32_t

static void buildRmtItemsForFrame(byte f) {
  byte* pat = &animation[f][0];
  for (int ch = 0; ch < 4; ++ch) {
    for (int pair = 0; pair < N_DIVS / 2; ++pair) {
      int d0 = pair * 2, d1 = d0 + 1;
      rmtItems[ch][pair].level0    = (pat[d0] >> ch) & 0x1;
      rmtItems[ch][pair].duration0 = stepDuration[d0];
      rmtItems[ch][pair].level1    = (pat[d1] >> ch) & 0x1;
      rmtItems[ch][pair].duration1 = stepDuration[d1];
    }
  }
}

static void writeRmtItemsAllChannels() {
  for (int ch = 0; ch < 4; ++ch) {
    // Explicitly stop each channel before handing it a new item list —
    // the legacy RMT driver doesn't reliably pick up new items while a
    // channel is still mid-loop from a previous write.
    rmt_tx_stop(rmtChannels[ch]);
    rmt_write_items(rmtChannels[ch], rmtItems[ch], N_DIVS / 2, false); // false = don't wait; loop runs in hardware
  }
}

static void setupRmtChannel(int idx, int gpioNum) {
  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX;
  config.channel = rmtChannels[idx];
  config.gpio_num = (gpio_num_t)gpioNum;
  config.clk_div = RMT_CLK_DIV;
  config.mem_block_num = 1;
  config.tx_config.loop_en = true;             // repeat the item list forever, in hardware
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_duty_percent = 50;  // unused, carrier disabled
  config.tx_config.carrier_freq_hz = 38000;    // unused, carrier disabled
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;

  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

// ---------------------------------------------------------------------
// Button handling — runs at a relaxed, non-time-critical pace, since
// the waveform itself is now entirely hardware-generated and no
// longer depends on software timing at all.
//
// buttonPressed[i] is true when that button IS pressed (normal sense).
// This differs from the original AVR code, which stored the raw PIND
// bit (true = NOT pressed, since INPUT_PULLUP idles high) and then
// used `!buttonPressed[i]` to test for an actual press. The logic
// below is equivalent, just inverted to a more intuitive convention —
// buttonPressed[i] here means exactly what its name says.
// ---------------------------------------------------------------------
static bool buttonPressed[N_BUTTONS];
static bool anyButtonPressed;
static unsigned long holdStartMs = 0;
static bool holding = false;

static void pollButtonsAndUpdateFrame() {
  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
    buttonPressed[i] = (digitalRead(buttonPins[i]) == LOW);
  }
  anyButtonPressed = false;
  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
    if (buttonPressed[i]) anyButtonPressed = true;
  }

  unsigned long now = millis();

  if (anyButtonPressed) {
    if (!holding) {
      holding = true;
      holdStartMs = now;
    } else if (now - holdStartMs >= BUTTON_HOLD_MS) {
      holdStartMs = now; // reset for auto-repeat while held

      byte newFrame = frame;
      if (buttonPressed[0]) {         // D2: previous frame
        newFrame = (frame < STEP_SIZE) ? (N_FRAMES - 1) : (frame - STEP_SIZE);
      } else if (buttonPressed[1]) {  // D3: next frame
        newFrame = (frame >= N_FRAMES - STEP_SIZE) ? 0 : (frame + STEP_SIZE);
      } else if (buttonPressed[2]) {  // D4: reset to frame 0
        newFrame = 0;
      }

      if (newFrame != frame) {
        frame = newFrame;
        buildRmtItemsForFrame(frame);
        writeRmtItemsAllChannels();
#ifdef BUTTON_DEBUG
        Serial.printf("frame changed -> %d\n", (int)frame);
#endif
      }
    }
  } else {
    holding = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200); // give the Serial Monitor time to connect

  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  computeStepDurations();

  // Resolve each Arduino pin macro (A0..A3) to its real GPIO number —
  // needed on boards like the Nano ESP32, which remaps Arduino pin
  // numbers away from raw GPIO numbers.
  int gpio[4] = {
    digitalPinToGPIONumber(OUT_PIN_0),
    digitalPinToGPIONumber(OUT_PIN_1),
    digitalPinToGPIONumber(OUT_PIN_2),
    digitalPinToGPIONumber(OUT_PIN_3)
  };
  for (int i = 0; i < 4; ++i) {
    setupRmtChannel(i, gpio[i]);
  }

  buildRmtItemsForFrame(frame);
  writeRmtItemsAllChannels();

  Serial.printf(
    "RMT waveform running: period=%lu ticks (%.3fus, %.1fHz)\n"
    "step durations (ticks): ",
    (unsigned long)PERIOD_TICKS,
    (double)PERIOD_TICKS * 1000.0 / RMT_TICK_HZ * 1000.0,
    (double)RMT_TICK_HZ / PERIOD_TICKS);
  for (int d = 0; d < N_DIVS; ++d) Serial.printf("%lu ", (unsigned long)stepDuration[d]);
  Serial.println();
}

void loop() {
  pollButtonsAndUpdateFrame();
  delay(1); // ~1ms button poll rate — plenty fast for human button presses
}
