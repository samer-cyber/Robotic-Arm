#pragma once
#include "ArmConfig.h"
#include <TMCStepper.h>

// ===========================================================================
//  The TMC2209 UART bus.
//
//  One wire, half duplex, up to four drivers addressed by their MS1/MS2
//  straps. Carries configuration and telemetry only - never motion. Step
//  pulses go down STEP/DIR; this bus decides how the driver interprets them.
// ===========================================================================
class TmcBus {
public:
  void begin(Print& log);

  bool probe(uint8_t i);          // re-test the link, updates online[]
  bool ensure(uint8_t i);         // online, or re-probe AND reconfigure
  void applyConfig(uint8_t i);    // push JCFG[i] into the chip
  void applyAll();

  // --- direction ----------------------------------------------------------
  // The DIR pins on this hardware are damaged, so rotation direction is the
  // GCONF.shaft bit and nothing else. Written only when the direction
  // actually inverts, and read back to confirm it landed: a silent
  // direction failure on an open-loop arm means driving the wrong way into
  // a hard stop, and nothing downstream would notice.
  bool setDirection(uint8_t i, bool positive);
  bool directionKnown(uint8_t i) const { return i < STEPPER_COUNT && dirKnown[i]; }
  bool directionPositive(uint8_t i) const { return i < STEPPER_COUNT && dirPos[i]; }
  uint32_t directionWrites() const { return dirWrites; }
  int  readShaftBit(uint8_t i);   // -1 = could not read

  bool online(uint8_t i) const { return i < STEPPER_COUNT && alive[i]; }
  bool allOnline() const;

  void report(Print& out, uint8_t i);       // full decoded status
  void reportBrief(Print& out, uint8_t i);  // one line, for streaming

  TMC2209Stepper* chip(uint8_t i) { return (i < STEPPER_COUNT) ? drv[i] : nullptr; }

  // Raw bus diagnostics, bypassing the library entirely.
  void echoTest(Print& out);
  void addressScan(Print& out);

private:
  bool writeShaft(uint8_t i, bool shaftBit);

  TMC2209Stepper* drv[STEPPER_COUNT] = {nullptr};
  bool alive[STEPPER_COUNT] = {false};
  bool dirPos[STEPPER_COUNT] = {};      // true = counting up
  bool dirKnown[STEPPER_COUNT] = {};    // have we ever written it?
  uint32_t dirWrites = 0;
};

extern TmcBus tmcBus;

// Microstepping straight through CHOPCONF.MRES.
//
// TMCStepper's own microsteps() helpers are not trustworthy here: the setter
// has no case for full step, so microsteps(1) silently does nothing and leaves
// the chip at 256, and the getter has been observed returning 0. That mismatch
// made every commanded move come out 256x too small, which reads as a
// mechanical fault. MRES is one nibble - own it.
void     tmcSetMicrosteps(TMC2209Stepper& t, uint16_t microsteps);
uint8_t  tmcReadMres(TMC2209Stepper& t);          // raw 0..15
uint16_t tmcReadMicrosteps(TMC2209Stepper& t);    // decoded, 0 if invalid
