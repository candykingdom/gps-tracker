#pragma once

#ifdef ARDUINO

#include <Arduino.h>

#else 

// Not Arduino (native tests)

#include <cstdint>
#include <assert.h>
#include <cstdio>

#include "fake-serial.h"

extern FakeSerial Serial2;

// These pins are used in the radio
static constexpr int PA15 = 15;
static constexpr int PB6 = 16 + 6;
static constexpr int PB7 = 16 + 7;
static constexpr int PC6 = 16 * 2 + 6;


#endif // ARDUINO
