/*
  TinyLev firmware — ESP32 (Arduino) port
  ----------------------------------------
  Ported from an AVR/ATmega328 (Arduino Nano) version that used direct
  PORTC/PIND register access, Timer1 PWM registers, and hand-tuned NOP
  delay loops. None of that exists on ESP32, so this version uses:

    - direct GPIO register writes for the phase-output nibble (a single
      register write per step instead of 4x digitalWrite() calls, which
      were far too slow to fit the 40kHz budget)
    - digitalRead for the buttons
    - a CPU-cycle busy-wait (ESP.getCycleCount()) instead of NOP counts
    - disableLoopWDT() because this sketch never returns from setup()
      and never yields, which would otherwise trip the watchdog and
      reboot the board.

  NOTE ON THE SYNC SIGNAL: the original AVR version generated its own
  40kHz reference on one pin (via Timer1) and read it back on another
  pin, requiring a jumper wire between them. Since the ESP32 is the one
  generating that signal in the first place, there's no need to loop it
  back through a GPIO at all — we just track the same 25us period
  directly against ESP.getCycleCount(). This removes the jumper wire
  and both sync pins entirely.

  *** PER-STEP TIMING (no manual tuning needed) ***
  Earlier versions of this port used hand-guessed WAIT_LIT/MID/LOT
  cycle counts, then a two-stage "measure fixed cost, add proportional
  delay" calibration. Both were indirect enough to drift: the second
  approach in particular could end up bursting all 24 steps out early
  in the period and then holding flat for the rest, which distorted
  the output waveform and caused an audible artifact.

  This version instead paces each of the 24 steps against its own
  absolute deadline (in CPU cycles from the start of the 25us period),
  computed once in setupStepOffsets(). This is the same self-correcting
  technique already used for the outer 40kHz loop: whatever time button
  reads or the register writes themselves take just eats into the gap
  before the next step's deadline, rather than accumulating drift or
  bunching transitions together. The 24 steps keep the same relative
  LIT:MID:LOT proportions the original AVR NOP counts used (9:13:14),
  scaled to fill exactly one 40kHz period.
*/

#include "soc/gpio_struct.h"

#define N_PORTS 1
#define N_DIVS 24
#define N_FRAMES 24
#define N_BUTTONS 6
#define STEP_SIZE 1
#define BUTTON_SENS 2500
#define SYNC_FREQ_HZ 40000

// Uncomment to print button-state and frame-change debug info to Serial.
// Comment out again once buttons are confirmed working — the extra
// Serial.printf() calls will occasionally cause a 40kHz overrun.
// #define BUTTON_DEBUG

// Uncomment to print the periodic "cycle budget: used/total (margin)"
// report roughly once a second. Comment out for normal use — the
// Serial.printf() call itself causes a brief overrun on the iteration
// it fires on.
// #define TIMING_DEBUG

// Relative proportions from the original AVR NOP-tuned delays.
#define WEIGHT_LIT 9
#define WEIGHT_MID 13
#define WEIGHT_LOT 14
// How many times each tier is used per 24-step pass (see runIterationBody()).
#define COUNT_LIT 1
#define COUNT_MID 7
#define COUNT_LOT 15

// ---------------------------------------------------------------------
// PIN CONFIGURATION — adjust these to match your ESP32 board's wiring.
// The four OUT pins carry the same 4-bit nibble that used to go to
// PORTC bits 0-3 (i.e. bit0..bit3 of animation[frame][d]).
// ---------------------------------------------------------------------
#define OUT_PIN_0   A0   // was PC0
#define OUT_PIN_1   A1   // was PC1
#define OUT_PIN_2   A2   // was PC2
#define OUT_PIN_3   A3   // was PC3

// D10/D11 are no longer used for sync — freed up for other purposes.

// Button pins (was PIND bits 2-7 / Arduino D2-D7)
static const uint8_t buttonPins[N_BUTTONS] = {2, 3, 4, 5, 6, 7};

static uint32_t CYCLES_LIT = 0, CYCLES_MID = 0, CYCLES_LOT = 0;

// ---------------------------------------------------------------------
// Write a 4-bit nibble to the four phase-output pins in one call, via
// direct GPIO set/clear registers (fast — see buildNibbleMasks()).
// ---------------------------------------------------------------------
static uint32_t nibbleSetMaskLo[16], nibbleClearMaskLo[16];
static uint32_t nibbleSetMaskHi[16], nibbleClearMaskHi[16];

static inline void writeNibble(uint8_t nibble) {
  GPIO.out_w1ts = nibbleSetMaskLo[nibble];
  GPIO.out_w1tc = nibbleClearMaskLo[nibble];
  GPIO.out1_w1ts.val = nibbleSetMaskHi[nibble];
  GPIO.out1_w1tc.val = nibbleClearMaskHi[nibble];
}

// Resolves each Arduino pin macro (A0..A3) to its real GPIO number
// (needed on boards like the Nano ESP32, which remaps Arduino pin
// numbers away from raw GPIO numbers) and builds set/clear bitmasks
// for every possible 4-bit nibble value.
static void buildNibbleMasks() {
  int8_t gpio[4] = {
    digitalPinToGPIONumber(OUT_PIN_0),
    digitalPinToGPIONumber(OUT_PIN_1),
    digitalPinToGPIONumber(OUT_PIN_2),
    digitalPinToGPIONumber(OUT_PIN_3)
  };

  for (int n = 0; n < 16; ++n) {
    uint32_t setLo = 0, clearLo = 0, setHi = 0, clearHi = 0;
    for (int bit = 0; bit < 4; ++bit) {
      int8_t g = gpio[bit];
      bool high = (n >> bit) & 0x1;
      if (g < 32) {
        if (high) setLo |= (1UL << g); else clearLo |= (1UL << g);
      } else {
        if (high) setHi |= (1UL << (g - 32)); else clearHi |= (1UL << (g - 32));
      }
    }
    nibbleSetMaskLo[n] = setLo;
    nibbleClearMaskLo[n] = clearLo;
    nibbleSetMaskHi[n] = setHi;
    nibbleClearMaskHi[n] = clearHi;
  }
}

#define OUTPUT_WAVE(pointer, d) writeNibble(pointer[(d) * N_PORTS + 0])

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
// Shared iteration state (used by both runIterationBody() and setup())
// ---------------------------------------------------------------------
static byte* emittingPointer;
static bool anyButtonPressed;
static bool buttonPressed[N_BUTTONS];
static short buttonCounter = 0;

// Absolute per-step target times (in CPU cycles from the start of the
// iteration) for each of the 24 output steps. Computed once in
// setupStepOffsets() below. stepOffset[0] is always 0 — step 0 fires
// immediately.
//
// Pacing each step against its own absolute deadline (rather than
// summing separately-measured "extra" delays after the fact, as an
// earlier version of this port did) means the 24 steps land evenly
// across the 25us period regardless of how much time button reads or
// the register writes themselves actually take — the same
// self-correcting trick already used for the outer 40kHz loop. This
// fixes a previous bug where all 24 steps were firing in a fast burst
// at the start of the period, then holding flat for the remainder —
// which produced a distorted, audible-sounding output instead of a
// clean stepped 40kHz phase-shifted wave.
static uint32_t stepOffset[N_DIVS];

// Builds stepOffset[] so the 24 steps are spaced with the same
// relative LIT:MID:LOT proportions the original AVR NOP-tuned delays
// used (9:13:14), scaled to fill exactly one 40kHz period.
static void setupStepOffsets(uint32_t cyclesPerPeriod) {
  const uint32_t totalWeight =
      COUNT_LIT * WEIGHT_LIT + COUNT_MID * WEIGHT_MID + COUNT_LOT * WEIGHT_LOT;
  const uint32_t unit = cyclesPerPeriod / totalWeight;
  CYCLES_LIT = unit * WEIGHT_LIT;
  CYCLES_MID = unit * WEIGHT_MID;
  CYCLES_LOT = unit * WEIGHT_LOT;

  uint32_t t = 0;
  stepOffset[0] = 0;
  t += CYCLES_LIT;               stepOffset[1] = t;   // after step0 (LIT)
  for (int d = 2; d <= 8; ++d) { t += CYCLES_MID; stepOffset[d] = t; } // 7x MID
  for (int d = 9; d <= 23; ++d) { t += CYCLES_LOT; stepOffset[d] = t; } // 15x LOT

  Serial.printf(
    "Step timing: period=%lu cycles, unit=%lu, LIT=%lu MID=%lu LOT=%lu, "
    "last step target=%lu (%.1f%% of period)\n",
    (unsigned long)cyclesPerPeriod, (unsigned long)unit,
    (unsigned long)CYCLES_LIT, (unsigned long)CYCLES_MID, (unsigned long)CYCLES_LOT,
    (unsigned long)stepOffset[23],
    100.0 * stepOffset[23] / cyclesPerPeriod);
}

// Runs one full 24-step output pass, samples the buttons, and (if held
// long enough) advances/reverses/resets the animation frame. Each step
// is paced against its own absolute deadline in stepOffset[]. Returns
// the number of CPU cycles the whole pass took (for the optional
// TIMING_DEBUG margin report).
static uint32_t runIterationBody() {
  uint32_t t0 = ESP.getCycleCount();

  for (uint8_t d = 0; d < N_DIVS; ++d) {
    while ((int32_t)(ESP.getCycleCount() - t0) < (int32_t)stepOffset[d]);
    OUTPUT_WAVE(emittingPointer, d);

    if (d == 0) {
      for (uint8_t i = 0; i < N_BUTTONS; ++i) {
        buttonPressed[i] = (digitalRead(buttonPins[i]) == LOW);
      }
    } else if (d == 1) {
      anyButtonPressed = false;
      for (uint8_t i = 0; i < N_BUTTONS; ++i) {
        if (buttonPressed[i]) anyButtonPressed = true;
      }
    }
  }

  if (anyButtonPressed) {
    ++buttonCounter;
#ifdef BUTTON_DEBUG
    if ((buttonCounter % 4000) == 0) { // throttle so it doesn't flood Serial
      Serial.printf("buttons: D2=%d D3=%d D4=%d D5=%d D6=%d D7=%d  counter=%d\n",
                    buttonPressed[0], buttonPressed[1], buttonPressed[2],
                    buttonPressed[3], buttonPressed[4], buttonPressed[5],
                    (int)buttonCounter);
    }
#endif
    if (buttonCounter > BUTTON_SENS) {
      buttonCounter = 0;

      // NOTE: previously these conditions were inverted (`!buttonPressed[i]`),
      // which fired the wrong action — e.g. holding D2 would trigger the
      // D3 (next-frame) branch instead of D2's (previous-frame) branch,
      // since D3 read as "not pressed" and satisfied that inverted check.
      if (buttonPressed[0]) {
        if (frame < STEP_SIZE) {
          frame = N_FRAMES - 1;
        } else {
          frame -= STEP_SIZE;
        }
      } else if (buttonPressed[1]) {
        if (frame >= N_FRAMES - STEP_SIZE) {
          frame = 0;
        } else {
          frame += STEP_SIZE;
        }
      } else if (buttonPressed[2]) {
        frame = 0;
      }
      emittingPointer = &animation[frame][0];
#ifdef BUTTON_DEBUG
      Serial.printf("frame changed -> %d\n", (int)frame);
#endif
    }
  } else {
    buttonCounter = 0;
  }

  return ESP.getCycleCount() - t0;
}

void setup()
{
  // This sketch never returns from setup() and never yields, so the
  // Arduino "loop task" watchdog must be disabled or the board will
  // reset itself with a "Task watchdog got triggered" panic.
  disableLoopWDT();

  pinMode(OUT_PIN_0, OUTPUT);
  pinMode(OUT_PIN_1, OUTPUT);
  pinMode(OUT_PIN_2, OUTPUT);
  pinMode(OUT_PIN_3, OUTPUT);
  buildNibbleMasks();
  writeNibble(0);

  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  Serial.begin(115200);
  delay(200); // give the Serial Monitor time to connect for calibration output

  emittingPointer = &animation[frame][0];

  const uint32_t cyclesPerPeriod =
      (uint32_t)((uint64_t)getCpuFrequencyMhz() * 1000000ULL / SYNC_FREQ_HZ);

  setupStepOffsets(cyclesPerPeriod);

  // --- Debug: periodically report how much of the 40kHz budget the
  // iteration actually used, so you can confirm there's margin. Prints
  // roughly once a second. Comment out once you've confirmed it's
  // stable — Serial.printf() itself takes far longer than one 25us
  // period, so the iteration it prints on will overrun (harmless in
  // isolation, but not something you want firing during real use).
  uint32_t debugCounter = 0;
  const uint32_t DEBUG_PRINT_INTERVAL = SYNC_FREQ_HZ; // ~1x/sec

  uint32_t nextCycleTarget = ESP.getCycleCount();

  LOOP:
    // Wait until the next 40kHz period boundary. Using an absolute,
    // accumulating target (rather than measuring a fresh interval each
    // time) avoids the loop slowly drifting relative to 40kHz.
    while ((int32_t)(ESP.getCycleCount() - nextCycleTarget) < 0);
    nextCycleTarget += cyclesPerPeriod;

    {
      uint32_t elapsed = runIterationBody();

#ifdef TIMING_DEBUG
      if (++debugCounter >= DEBUG_PRINT_INTERVAL) {
        debugCounter = 0;
        int32_t margin = (int32_t)cyclesPerPeriod - (int32_t)elapsed;
        Serial.printf("cycle budget: %lu used / %lu total (margin %ld)\n",
                      (unsigned long)elapsed, (unsigned long)cyclesPerPeriod, (long)margin);
      }
#else
      (void)elapsed;
#endif
    }

  goto LOOP;
}

void loop() {}
