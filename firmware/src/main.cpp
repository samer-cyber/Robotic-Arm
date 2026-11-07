#include <Arduino.h>
#include <esp_system.h>
#include "ArmConfig.h"
#include "Arm.h"
#include "TmcBus.h"
#include "Protocol.h"

static const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_EXT:       return "reset button";
    case ESP_RST_PANIC:     return "PANIC - firmware crashed";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "BROWNOUT - supply dipped";
    case ESP_RST_DEEPSLEEP: return "deep sleep wake";
    default:                return "unknown";
  }
}

void setup() {
  // The stock 256-byte RX buffer overflows when a block of commands arrives
  // while the firmware is printing. The bytes it drops are usually newlines,
  // so two commands fuse into one nonsense word and the failure looks random.
  Serial.setRxBufferSize(2048);
  Serial.begin(HOST_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);
  delay(150);

  Serial.println();
  Serial.printf("# %s %s proto=%u built %s %s\n",
                FW_NAME, FW_VERSION, PROTO_VERSION, __DATE__, __TIME__);
  Serial.printf("# reset: %s\n", resetReasonText());

  configLoadDefaults();
  Serial.println(configLoad() ? "# config restored from flash"
                              : "# no saved config - using defaults");

  tmcBus.begin(Serial);
  arm.begin(Serial);

  for (uint8_t i = 0; i < JOINT_COUNT; i++) {
    if (JCFG[i].kind == KIND_STEPPER)
      Serial.printf("# %s step=GPIO%-2u dir=UART-shaft en=GPIO%-2u addr=%u\n",
                    JCFG[i].name, JPIN[i].step,
                    JPIN[i].enable, JPIN[i].tmcAddr);
    else
      Serial.printf("# %s pwm=GPIO%-2u\n", JCFG[i].name, JPIN[i].step);
  }
  if (!pinAudit(Serial))
    Serial.println("# !! pin problems above - fix with setpin, then save and reboot");

  if (CFG_STALE)
    Serial.println("# !! saved config was an older layout and was DISCARDED. "
                   "Defaults are in use - check `config`, then `save`.");

  // A controller with no joints answers every command cheerfully and moves
  // nothing, which is the most confusing failure mode there is. Refuse to be
  // quiet about it.
  if (arm.attachedCount() == 0) {
    Serial.println("# !! NO JOINTS ATTACHED. Nothing will move, and `enable`");
    Serial.println("# !! will report success while doing nothing.");
    Serial.println("# !! Fix with:  defaults   then   save   then   reboot");
  } else {
    Serial.printf("# %u joints ready\n", arm.attachedCount());
  }

  proto.begin(Serial);
  Serial.println("# motors are DE-ENERGISED. `enable 1` to energise, `help` for commands.");
  Serial.println("ok");
}

void loop() {
  proto.poll();
  arm.poll();
}
