/*
  ESP32 port of the original AVR sketch
  ======================================
  WHY THINGS CHANGED (read this before flashing):

  1. 40 kHz sync signal
     The AVR version bit-banged Timer1 into fast-PWM mode by hand.
     On ESP32 the equivalent -- and far more precise -- approach is the
     LEDC peripheral: a hardware clock divider drives the pin directly,
     so the frequency is rock solid and completely independent of what
     the CPU/RTOS/WiFi stack is doing. That's what PIN_SYNC_OUT below
     uses.

  2. Port register access (PORTC / PIND / PINB)
     Those registers don't exist on ESP32. They're replaced with direct
     GPIO register access (GPIO_OUT_W1TS_REG / GPIO_OUT_W1TC_REG /
     GPIO_IN_REG) so the hot loop is still a handful of instructions,
     same as on the AVR. NOTE: this fast path only works for GPIO0-31 --
     all pins below are deliberately chosen from that range. If you
     rewire to a pin >=32 you must switch to the *_REG1 registers.

  3. The nop-counted delays (WAIT_LIT/MID/LOT)
     AVR nops are 1 cycle @16 MHz (62.5 ns) and the compiler emits
     exactly what you wrote. Neither is true on the ESP32 (240 MHz,
     dual core, cache, FreeRTOS preemption), so counting nops there is
     meaningless. Instead this version measures real elapsed CPU cycles
     with the hardware cycle counter (ESP.getCycleCount()) and busy-waits
     until the equivalent *time* (not instruction count) has passed --
     the original nop counts were converted to nanoseconds at 16 MHz and
     are re-scaled here to whatever the ESP32 is actually clocked at.

  4. Watchdog
     The original design blocks forever inside setup() via `goto LOOP`
     and never yields. On the ESP32 that starves the idle task on
     whichever core it runs on and the Task Watchdog will reboot the
     chip. The tight loop is therefore run as its own FreeRTOS task,
     pinned to one core, with that core's watchdog disabled.

  5. Pin numbers
     Picked to (a) stay in the 0-31 fast-GPIO bank and (b) avoid strapping
     pins (0/2/5/12/15) and the flash pins (6-11). Change PIN_* below to
     match your actual wiring -- these are just sane defaults for a
     generic ESP32 DevKit.

  For real (non-loopback) use, PIN_SYNC_IN should come from your actual
  index/position sensor rather than being wired straight back to
  PIN_SYNC_OUT -- the loopback wiring from the original comment is only
  useful for bench-testing the timing.
*/

#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"

// ---------------- USER-CONFIGURABLE PIN MAP ----------------
// 4-bit output nibble (was PORTC bits A0-A3 on the AVR)
#define PIN_OUT0     18
#define PIN_OUT1     19
#define PIN_OUT2     21
#define PIN_OUT3     22

#define PIN_SYNC_OUT 25   // hardware 40kHz PWM out (was AVR pin 10)
#define PIN_SYNC_IN  26   // sync in -- wire PIN_SYNC_OUT -> PIN_SYNC_IN for bench testing (was AVR pin 11)

#define PIN_ENC_CLK   4   // rotary encoder CLK (was AVR pin 2)
#define PIN_ENC_DT   13   // rotary encoder DT  (was AVR pin 3)
#define PIN_ENC_SW   27   // rotary encoder SW  (was AVR pin 4)

#define SYNC_FREQ_HZ 40000
#define LEDC_RESOLUTION_BITS 8     // only need 50% duty, 8 bits is plenty
#define RT_CORE 1                  // core the timing-critical loop runs on
// -------------------------------------------------------------

#define N_DIVS    24
#define N_FRAMES  24
#define STEP_SIZE 1
#define BUTTON_SENS 2500
#define ENCODER_DEBOUNCE_LOOPS 3

// Original AVR nop counts converted to nanoseconds @16 MHz (62.5ns/cycle),
// re-derived as ESP32 cycles at setup() time from the actual CPU clock.
#define WAIT_LIT_NS 562   //  9 cycles @16MHz
#define WAIT_MID_NS 812   // 13 cycles @16MHz
#define WAIT_LOT_NS 875   // 14 cycles @16MHz

static uint32_t cyclesLit, cyclesMid, cyclesLot;

static inline void delayCycles(uint32_t cycles) {
  uint32_t start = ESP.getCycleCount();
  while ((uint32_t)(ESP.getCycleCount() - start) < cycles) {
    // busy-wait -- intentional, this is the timing-critical path
  }
}

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

// Precomputed set/clear masks for every possible 4-bit nibble value (0-15),
// built once in setup() from the PIN_OUT* assignments so the hot loop just
// does two register writes per division, same as the AVR's single PORTC=.
static uint32_t setMask[16];
static uint32_t clearMask[16];

static uint32_t bitCLK, bitDT, bitSW, bitSyncIn;

static inline void outputNibble(byte v) {
  REG_WRITE(GPIO_OUT_W1TC_REG, clearMask[v]);
  REG_WRITE(GPIO_OUT_W1TS_REG, setMask[v]);
}

// ---------------------------------------------------------------
// Timing-critical loop -- runs forever, pinned to its own core.
// This is the direct equivalent of the AVR's `goto LOOP;` block.
// ---------------------------------------------------------------
void rtLoopTask(void *pv) {
  byte* emittingPointer = &animation[frame][0];
  uint32_t buttonsIn;

  bool encoderPin[3]; // [0]=CLK [1]=DT [2]=SW
  short buttonCounter = 0;

  byte clkReadState    = (REG_READ(GPIO_IN_REG) & bitCLK) ? 1 : 0;
  byte lastCLK          = clkReadState;
  byte clkDebounceCount = 0;

  for (;;) {
    // wait for sync line to go low (equivalent of while(PINB & ...))
    while (REG_READ(GPIO_IN_REG) & bitSyncIn) { }

    buttonsIn = REG_READ(GPIO_IN_REG);

    outputNibble(emittingPointer[0]);                                delayCycles(cyclesLit);
    outputNibble(emittingPointer[1]);                                delayCycles(cyclesMid);
    outputNibble(emittingPointer[2]); encoderPin[0] = buttonsIn & bitCLK; delayCycles(cyclesMid);
    outputNibble(emittingPointer[3]); encoderPin[1] = buttonsIn & bitDT;  delayCycles(cyclesMid);
    outputNibble(emittingPointer[4]); encoderPin[2] = buttonsIn & bitSW;  delayCycles(cyclesMid);
    outputNibble(emittingPointer[5]);                                delayCycles(cyclesMid);
    outputNibble(emittingPointer[6]);                                delayCycles(cyclesMid);
    outputNibble(emittingPointer[7]);                                delayCycles(cyclesMid);
    for (int d = 8; d <= 22; ++d) {
      outputNibble(emittingPointer[d]);
      delayCycles(cyclesLot);
    }
    outputNibble(emittingPointer[23]);

    // --- ROTATION: debounce CLK over a few loop passes, read DT at the edge ---
    {
      byte curCLK = encoderPin[0];

      if (curCLK == clkReadState) {
        if (clkDebounceCount < 255) ++clkDebounceCount;
      } else {
        clkReadState = curCLK;
        clkDebounceCount = 0;
      }

      if (clkDebounceCount == ENCODER_DEBOUNCE_LOOPS && clkReadState != lastCLK) {
        if (clkReadState == 0) { // confirmed CLK falling edge -> one detent
          byte curDT = encoderPin[1];
          if (curDT != clkReadState) {
            // DT HIGH while CLK LOW: clockwise -> down
            if (frame < STEP_SIZE) {
              frame = N_FRAMES - 1;
            } else {
              frame -= STEP_SIZE;
            }
          } else {
            // DT LOW while CLK LOW: counter-clockwise -> up
            if (frame >= N_FRAMES - STEP_SIZE) {
              frame = 0;
            } else {
              frame += STEP_SIZE;
            }
          }
          emittingPointer = &animation[frame][0];
        }
        lastCLK = clkReadState;
      }
    }

    // --- SW (encoder push button): hold to reset to frame 0 ---
    if (!encoderPin[2]) {
      ++buttonCounter;
      if (buttonCounter > BUTTON_SENS) {
        buttonCounter = 0;
        frame = 0;
        emittingPointer = &animation[frame][0];
      }
    } else {
      buttonCounter = 0;
    }
  }
}

void setup() {
  pinMode(PIN_OUT0, OUTPUT);
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  digitalWrite(PIN_OUT0, LOW);
  digitalWrite(PIN_OUT1, LOW);
  digitalWrite(PIN_OUT2, LOW);
  digitalWrite(PIN_OUT3, LOW);

  pinMode(PIN_SYNC_IN, INPUT);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);

  Serial.begin(115200);

  // Build the set/clear bitmasks for every 4-bit nibble value once.
  const int outPins[4] = {PIN_OUT0, PIN_OUT1, PIN_OUT2, PIN_OUT3};
  for (int v = 0; v < 16; ++v) {
    uint32_t s = 0, c = 0;
    for (int b = 0; b < 4; ++b) {
      if (v & (1 << b)) s |= (1UL << outPins[b]);
      else               c |= (1UL << outPins[b]);
    }
    setMask[v] = s;
    clearMask[v] = c;
  }

  bitCLK    = 1UL << PIN_ENC_CLK;
  bitDT     = 1UL << PIN_ENC_DT;
  bitSW     = 1UL << PIN_ENC_SW;
  bitSyncIn = 1UL << PIN_SYNC_IN;

  // Re-derive the original AVR nop-timings as ESP32 cycle counts, scaled
  // to whatever clock speed this chip is actually running at.
  uint64_t cpuHz = (uint64_t)getCpuFrequencyMhz() * 1000000ULL;
  cyclesLit = (uint32_t)((uint64_t)WAIT_LIT_NS * cpuHz / 1000000000ULL);
  cyclesMid = (uint32_t)((uint64_t)WAIT_MID_NS * cpuHz / 1000000000ULL);
  cyclesLot = (uint32_t)((uint64_t)WAIT_LOT_NS * cpuHz / 1000000000ULL);

  // Precise hardware 40kHz sync out via LEDC (Arduino-ESP32 core v3.x API).
  // If you're on core v2.x, replace these two lines with:
  //   ledcSetup(0, SYNC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  //   ledcAttachPin(PIN_SYNC_OUT, 0);
  //   ledcWrite(0, 1 << (LEDC_RESOLUTION_BITS - 1));
  ledcAttach(PIN_SYNC_OUT, SYNC_FREQ_HZ, LEDC_RESOLUTION_BITS);
  ledcWrite(PIN_SYNC_OUT, 1 << (LEDC_RESOLUTION_BITS - 1)); // 50% duty

  // The real-time task below never yields, so stop the watchdog from
  // panicking and pin it to one dedicated core.
  disableCore0WDT();
  disableCore1WDT();

  xTaskCreatePinnedToCore(
    rtLoopTask, "rt_loop", 4096, NULL,
    configMAX_PRIORITIES - 1, NULL, RT_CORE
  );
}

void loop() {
  // Everything happens in rtLoopTask; nothing to do here.
  vTaskDelay(portMAX_DELAY);
}
