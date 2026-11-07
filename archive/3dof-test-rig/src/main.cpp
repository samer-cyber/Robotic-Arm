#include <Arduino.h>
#include "config.h"
#include "drivers.h"
#include "motion.h"
#include "console.h"
#include "servos.h"
#include <esp_system.h>

void setup() {
  // The default 256-byte RX buffer overflows when you paste a block of
  // commands while the firmware is busy printing, and the dropped bytes are
  // usually newlines - so two commands silently fuse into one nonsense word.
  Serial.setRxBufferSize(2048);
  Serial.begin(HOST_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) { delay(10); }
  delay(200);

  Serial.println();
  Serial.printf("# %s %s  (%s %s)\n", FW_NAME, FW_VERSION, __DATE__, __TIME__);

  // Why did we just start? An unexplained reset in the middle of a move is a
  // symptom, and the chip already knows the answer - so print it rather than
  // leave anyone guessing at loose wires.
  const char *why = "?";
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  why = "power on";                                break;
    case ESP_RST_SW:       why = "software restart (reboot command)";       break;
    case ESP_RST_EXT:      why = "external reset (the button)";             break;
    case ESP_RST_PANIC:    why = "PANIC - firmware crashed, see the trace"; break;
    case ESP_RST_INT_WDT:  why = "interrupt watchdog";                      break;
    case ESP_RST_TASK_WDT: why = "task watchdog - something blocked too long"; break;
    case ESP_RST_WDT:      why = "watchdog";                                break;
    case ESP_RST_BROWNOUT: why = "BROWNOUT - supply dipped. Short, or too "
                                 "much current drawn from the 3V3 rail";    break;
    case ESP_RST_DEEPSLEEP:why = "deep sleep wake";                         break;
    default:               why = "unknown";                                 break;
  }
  Serial.printf("# reset reason: %s\n", why);

  cfgLoadDefaults();
  if (cfgLoad()) Serial.println("# config restored from NVS");
  else           Serial.println("# no saved config, using defaults - run cfg and fix the geometry");

  delay(300);            // let VS and the drivers' internal rails settle
  drivers.begin();
  for (uint8_t i = 0; i < N_AXES; i++) {
    Serial.printf("# %s driver addr %u bus %u : %s\n",
                  AX[i].name, PINS[i].addr, PINS[i].bus,
                  drivers.online[i] ? "ONLINE"
                                    : "NO REPLY - check UART, VIO and MS1/MS2 straps");
  }
  if (!drivers.allOnline())
    Serial.println("# !! Some drivers did not answer. They are on power-up "
                   "defaults (256 microsteps, VREF current), so moves will be "
                   "tiny and weak. Fix the bus, then run: probe");

  for (uint8_t i = 0; i < N_AXES; i++) {
    Serial.printf("# %s pins: STEP GPIO%-2u  DIR GPIO%-2u  EN GPIO%-2u\n",
                  AX[i].name, PINS[i].step, PINS[i].dir, PINS[i].en);
  }
  pinAudit(Serial);

  motion.begin(Serial);
  servos.begin();
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    Serial.printf("# %-4s servo GPIO%-2u  %.0f deg range, limits %.0f..%.0f",
                  SV[i].name, SV[i].pin, SV[i].rangeDeg, SV[i].minDeg, SV[i].maxDeg);
    Serial.println();
  }
  Serial.println("# servos are LIVE and holding their park position. `svoff` makes them limp.");
  Serial.println("# !! servo power must NOT come from the ESP32 5V pin - see docs/WIRING.md");
  console.begin(Serial);

  Serial.println("# drivers are DISABLED. `en 1` to energise, `?` for help.");
  Serial.println("ok");
}

void loop() {
  console.poll();
  motion.poll();
}
