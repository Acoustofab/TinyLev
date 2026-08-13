/*
  Acoustic Levitator (UpnaLab-style, L298N driver) -- ESP32 / Nano ESP32
  ========================================================================
  This replaces the original AVR sketch's approach entirely, once you
  realize what it's actually doing: the 4-bit "PORTC" nibble it wrote at
  40kHz was driving an L298N's IN1-IN4 pins -- IN1/IN2 as a complementary
  pair for the TOP transducer array, IN3/IN4 as a complementary pair for
  the BOTTOM array -- with the phase between the two pairs adjustable by
  the rotary encoder in 15-degree steps (24 steps x 15 deg = 360 deg).
  That's how the standing wave / levitation node gets tuned.

  On the AVR, generating two independently-phased 40kHz square waves
  meant bit-banging PORTC with nop-counted delays, paced by a self-
  looped-back Timer1 PWM signal (the "connect D10 to D11" jumper).

  On the ESP32 there's a much better tool for exactly this job: the LEDC
  peripheral can run two output channels off one shared hardware timer
  and give each an independent phase offset (the "hpoint" register) --
  fully hardware generated, glitch-free, and updated atomically. So:

    - IN1/IN2 (top array)    -> LEDC channels locked at 0 deg / 180 deg,
                                fixed forever as the phase reference.
    - IN3/IN4 (bottom array) -> LEDC channels whose phase is nudged by
                                the encoder; the ISR-free "loop" now just
                                writes one new hpoint value per detent.

  There is no more bit-banged output loop, no more nop delays, and no
  more sync-pin loopback -- the hardware timer runs the actual 40kHz
  drive continuously and correctly by itself. The rotary encoder only
  needs to be polled at ordinary, non-time-critical speed in loop().

  WIRING (Nano ESP32 board labels -- see note below on why D*///A* and not
 // raw GPIO numbers):
   // D2  -> L298N IN1
   // D3  -> L298N IN2
   // D4  -> L298N IN3
    //D5  -> L298N IN4
    //A0  -> encoder CLK
    //A1  -> encoder DT
    //A2  -> encoder SW
    //GND -> common ground with the L298N logic side
 // Make sure the L298N's ENA/ENB jumpers are in place (or tied high) so
  //both H-bridges are enabled -- this sketch doesn't drive them.
  //The old D10/D11 jumper from the Instructable's simple version is not
  //needed with this design.

 // Nano ESP32 note: this board runs pinMode()/digitalWrite() through a translation layer where the numbers you type are the *Arduino* pin labels (D2, A0, ...),
 // not the chip's real GPIO numbers. The LEDC ESP-IDF driver used here needs the real GPIO number, so setup() resolves it itself via digitalPinToGPIONumber() -- don't hardcode raw GPIO numbers for this board.

#include <Arduino.h>
#include "driver/ledc.h"

// ---------------- USER-CONFIGURABLE PIN MAP ----------------
#define PIN_IN1 D2   // top array, channel A
#define PIN_IN2 D3   // top array, channel A (complementary)
#define PIN_IN3 D4   // bottom array, channel B
#define PIN_IN4 D5   // bottom array, channel B (complementary)

#define PIN_ENC_CLK A0
#define PIN_ENC_DT  A1
#define PIN_ENC_SW  A2
// -------------------------------------------------------------

#define SYNC_FREQ_HZ 40000
#define N_FRAMES     24     // phase steps around a full 360 deg turn
#define STEP_SIZE    1
#define ENCODER_DEBOUNCE_LOOPS 3
#define LONG_PRESS_MS 700    // hold SW this long to reset phase to 0 deg

#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_NUM  LEDC_TIMER_0
// 10-bit resolution is the practical ceiling for a clean 40kHz timer off
// the 80MHz APB clock (11-bit would need a divisor < 1, which LEDC can't
// do) -- still gives ~24ns phase-step resolution, far finer than needed.
#define LEDC_RES_BITS   LEDC_TIMER_10_BIT

#define CH_IN1 LEDC_CHANNEL_0
#define CH_IN2 LEDC_CHANNEL_1
#define CH_IN3 LEDC_CHANNEL_2
#define CH_IN4 LEDC_CHANNEL_3

static volatile int frame = 0;
static uint32_t maxDuty;
static uint32_t halfDuty;

// Push the encoder's current phase step out to the hardware. Only the
// bottom array's channels move; the top array stays fixed as reference.
static void applyPhase(int f) {
  uint32_t phaseOffset = ((uint32_t)f * maxDuty + N_FRAMES / 2) / N_FRAMES; // rounded
  uint32_t in4H = (phaseOffset + halfDuty) % maxDuty;
  ledc_set_duty_and_update(LEDC_SPEED_MODE, CH_IN3, halfDuty, phaseOffset);
  ledc_set_duty_and_update(LEDC_SPEED_MODE, CH_IN4, halfDuty, in4H);
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);

  ledc_timer_config_t timerCfg = {};
  timerCfg.speed_mode      = LEDC_SPEED_MODE;
  timerCfg.duty_resolution = LEDC_RES_BITS;
  timerCfg.timer_num       = LEDC_TIMER_NUM;
  timerCfg.freq_hz         = SYNC_FREQ_HZ;
  timerCfg.clk_cfg         = LEDC_AUTO_CLK;
  if (ledc_timer_config(&timerCfg) != ESP_OK) {
    Serial.println("FATAL: LEDC timer config failed (couldn't hit 40kHz "
                    "at this resolution) -- lower LEDC_RES_BITS.");
    while (true) { delay(1000); }
  }

  maxDuty  = 1UL << LEDC_RES_BITS;
  halfDuty = maxDuty / 2;

  struct { ledc_channel_t ch; int pin; uint32_t hpoint; } chans[4] = {
    { CH_IN1, PIN_IN1, 0 },
    { CH_IN2, PIN_IN2, halfDuty },
    { CH_IN3, PIN_IN3, 0 },          // updated immediately below by applyPhase()
    { CH_IN4, PIN_IN4, halfDuty },
  };

  for (auto &c : chans) {
    int gpio = digitalPinToGPIONumber(c.pin);
    if (gpio < 0 || gpio > 48) {
      Serial.printf("FATAL: pin did not resolve to a valid GPIO (got %d)\n", gpio);
      while (true) { delay(1000); }
    }
    ledc_channel_config_t chCfg = {};
    chCfg.gpio_num   = gpio;
    chCfg.speed_mode = LEDC_SPEED_MODE;
    chCfg.channel    = c.ch;
    chCfg.timer_sel  = LEDC_TIMER_NUM;
    chCfg.duty       = halfDuty;
    chCfg.hpoint     = c.hpoint;
    ledc_channel_config(&chCfg);
  }

  applyPhase(frame); // start at 0 deg (in-phase); turn the encoder to move toward 180 deg
}

void loop() {
  static byte clkReadState    = digitalRead(PIN_ENC_CLK);
  static byte lastCLK          = clkReadState;
  static byte clkDebounceCount = 0;
  static unsigned long swPressStart = 0;
  static bool swHeld = false;
  static bool swConsumed = false;

  // --- rotary encoder: adjust phase in +/-15 deg steps ---
  byte curCLK = digitalRead(PIN_ENC_CLK);
  if (curCLK == clkReadState) {
    if (clkDebounceCount < 255) ++clkDebounceCount;
  } else {
    clkReadState = curCLK;
    clkDebounceCount = 0;
  }

  if (clkDebounceCount == ENCODER_DEBOUNCE_LOOPS && clkReadState != lastCLK) {
    if (clkReadState == LOW) { // confirmed CLK falling edge -> one detent
      byte curDT = digitalRead(PIN_ENC_DT);
      if (curDT != clkReadState) {
        frame = (frame - STEP_SIZE + N_FRAMES) % N_FRAMES;
      } else {
        frame = (frame + STEP_SIZE) % N_FRAMES;
      }
      applyPhase(frame);
      Serial.printf("phase = %d deg (frame %d/%d)\n", frame * 360 / N_FRAMES, frame, N_FRAMES);
    }
    lastCLK = clkReadState;
  }

  // --- push button: hold to reset phase to 0 deg ---
  if (digitalRead(PIN_ENC_SW) == LOW) {
    if (!swHeld) {
      swHeld = true;
      swConsumed = false;
      swPressStart = millis();
    } else if (!swConsumed && millis() - swPressStart > LONG_PRESS_MS) {
      frame = 0;
      applyPhase(frame);
      Serial.println("phase reset to 0 deg");
      swConsumed = true; // don't re-trigger again until released
    }
  } else {
    swHeld = false;
  }

  delay(1); // encoder is mechanical, no need to poll faster than this
}
