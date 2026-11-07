#pragma once
#include "config.h"
#include <TMCStepper.h>

// Which HardwareSerial carries which driver bus. Board-specific, because the
// classic ESP32 and the S3 disagree about which UART is free on which pins.
HardwareSerial* busPort(uint8_t bus);

// Low-level UART diagnostics. Bypasses TMCStepper entirely and shows you what
// is actually on the wire. See `uart` in the console help.
void uartDiag(Stream &o, int argc, char **argv);

// Microstepping straight through CHOPCONF.MRES, bypassing TMCStepper's
// microsteps() helpers. Its setter has no case for 1 and its getter has been
// seen returning 0 for full step, so neither is trustworthy as a source of
// truth. MRES is one nibble and the mapping is trivial - own it here.
void     tmcSetMicrosteps(TMC2209Stepper &t, uint16_t us);
uint8_t  tmcMres(TMC2209Stepper &t);          // raw 0..15, 0=256 ... 8=full
uint16_t tmcGetMicrosteps(TMC2209Stepper &t); // decoded, 0 if MRES is invalid

class Drivers {
public:
  void begin();
  void applyAll();
  void apply(uint8_t i);            // push AX[i] settings into the chip
  bool probe(uint8_t i);            // re-test UART link, updates online[]
  bool ensure(uint8_t i);           // online, or re-probe and reconfigure
  bool allOnline();
  void report(Stream &s, uint8_t i);        // decoded DRV_STATUS + settings
  void reportLine(Stream &s, uint8_t i);    // one-line live telemetry
  void reportAll(Stream &s);

  TMC2209Stepper* d(uint8_t i) { return (i < N_AXES) ? tmc[i] : nullptr; }
  bool online[MAX_AXES] = {false};

private:
  TMC2209Stepper* tmc[MAX_AXES] = {nullptr};
};

extern Drivers drivers;
