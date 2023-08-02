#pragma once

#include "RadioLib.h"
#include "types.h"

#ifndef ARDUINO
#include "native-hal.h"
#endif

class Radio {
 public:
  bool Begin();

 private:
  static constexpr int kRadioCs = PA15;
  static constexpr int kRadioDio1 = PB6;
  static constexpr int kRadioBusy = PB7;
  static constexpr int kRadioRxen = PC6;

#ifdef ARDUINO
  SPISettings radio_spi_settings_{10 * 1000 * 1000, MSBFIRST, SPI_MODE0};
  SX1262 radio_ = new Module(kRadioCs, kRadioDio1, /*rst=*/RADIOLIB_NC,
                             /*gpio=*/kRadioBusy, SPI, radio_spi_settings_);
#else  // ARDUINO
  NativeHal hal_;
  SX1262 radio_ = new Module(&hal_, kRadioCs, kRadioDio1, /*rst=*/RADIOLIB_NC,
                             /*gpio=*/kRadioBusy);
#endif
};
