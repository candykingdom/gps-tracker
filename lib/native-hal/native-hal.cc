#include "native-hal.h"

uint32_t NativeHal::digitalRead(uint32_t pin) {
  // TODO: implement this?
  return 0;
}

unsigned long NativeHal::millis() { return millis_; }

unsigned long NativeHal::micros() { return micros_; }

void NativeHal::setMillis(unsigned long millis) { millis_ = millis; }

void NativeHal::setMicros(unsigned long micros) { micros_ = micros; }
