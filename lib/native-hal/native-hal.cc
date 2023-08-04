#include "native-hal.h"

// #define SPI_DEBUG

#ifdef SPI_DEBUG
#define spi_printf printf
#else
#define spi_printf  //
#endif

#include <assert.h>

#include <cstdio>

uint32_t NativeHal::digitalRead(uint32_t pin) {
  // TODO: implement this?
  return 0;
}

unsigned long NativeHal::millis() { return millis_; }

unsigned long NativeHal::micros() { return micros_; }

void NativeHal::setMillis(unsigned long millis) { millis_ = millis; }

void NativeHal::setMicros(unsigned long micros) { micros_ = micros; }

void NativeHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
  if (len == 0) {
    spi_printf("Warning: spiTransfer called with len=0\n");
  }

  if (out[0] == RADIOLIB_SX126X_CMD_READ_REGISTER) {
    assert(len >= 3);
    uint16_t reg = (out[1] << 8) + out[2];
    if (reg == RADIOLIB_SX126X_REG_VERSION_STRING) {
      spi_printf("SPI version read\n");
      // First four positions are used by writing out the command
      strcpy((char*)in, "    SX1261");
    } else {
      spi_printf("SPI read of register: %04X\n", reg);
    }
  } else if (out[0] == RADIOLIB_SX126X_CMD_WRITE_REGISTER) {
    assert(len >= 3);
    uint16_t reg = (out[1] << 8) + out[2];
    spi_printf("SPI write. Register: %04X, data: ", reg);
    for (size_t i = 3; i < len; i++) {
      spi_printf("%02X ", out[i]);
    }
    spi_printf("\n");
  } else if (out[0] == RADIOLIB_SX126X_CMD_GET_STATUS) {
    spi_printf("SPI get status\n");
    assert(len == 2);
  } else if (out[0] == RADIOLIB_SX126X_CMD_GET_PACKET_TYPE) {
    spi_printf("SPI get packet type\n");
    assert(len == 3);
    in[2] = RADIOLIB_SX126X_PACKET_TYPE_LORA;
  } else {
    spi_printf("spiTransfer: ");
    for (size_t i = 0; i < len; i++) {
      spi_printf("%02X ", out[i]);
    }
    spi_printf("\n");
  }
}