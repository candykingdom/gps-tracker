#pragma once

#include <array>

#include "RadioLib.h"
#include "types.h"

#ifndef ARDUINO
#include "native-hal.h"
#endif

enum class RadioOperation {
  kNone,
  kTransmit,
  kReceive,
};

class Radio {
 public:
  Radio() {}
  bool Begin();

  void Step();
  bool IsIdle();

  int16_t StartTransmit(uint8_t* data, size_t len);

  int16_t ReceivedPacketLength();
  uint8_t* GetPacketBuffer();

  int16_t StartReceive();

#ifndef ARDUINO
  // For testing
  NativeHal& GetHal();
#endif  // ARDUINO

 private:
  static constexpr int kRadioCs = PA15;
  static constexpr int kRadioDio1 = PB6;
  static constexpr int kRadioBusy = PB7;
  static constexpr int kRadioRxen = PC6;

  RadioOperation prev_op_ = RadioOperation::kNone;

  static constexpr size_t kPacketBufferSize = 30;
  uint8_t packet_buffer_[kPacketBufferSize] = {0};
  int16_t packet_length_ = 0;

#ifdef ARDUINO
  SPISettings radio_spi_settings_{10 * 1000 * 1000, MSBFIRST, SPI_MODE0};
  SX1262 radio_ = new Module(kRadioCs, kRadioDio1, /*rst=*/RADIOLIB_NC,
                             /*gpio=*/kRadioBusy, SPI, radio_spi_settings_);
#else  // ARDUINO
  NativeHal hal_;
  Module module_{&hal_, kRadioCs, kRadioDio1, /*rst=*/RADIOLIB_NC,
                 /*gpio=*/kRadioBusy};
  SX1262 radio_ = SX1262(&module_);
#endif
};
