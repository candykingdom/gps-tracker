#include "radio.h"

// Not a class member because `setDio1Action` takes a C-style function
volatile bool radio_idle = false;

void SetRadioIdle() { radio_idle = true; }

bool Radio::Begin() {
  Serial2.print("Initializing radio... ");
  int state = radio_.begin(/*freq=*/915.0, /*bw*/ 125, /*sf=*/9, /*cr=*/7,
                           /*syncWord=*/18, /*power=*/22, /*preambleLength=*/8,
                           /*txcoVoltage=*/1.8, /*useRegulatorLdo=*/false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial2.println("success.");
  } else {
    Serial2.printf("failed, code: %d\n", state);
    return false;
  }

  radio_.setDio1Action(SetRadioIdle);
  radio_.setRfSwitchPins(kRadioRxen, RADIOLIB_NC);
  state = radio_.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    Serial2.printf("`radio_.setDio2AsRfSwitch` failed: %d\n", state);
    return false;
  }

  // TODO: start receiving?

  return true;
}