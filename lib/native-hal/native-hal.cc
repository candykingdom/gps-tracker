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

void NativeHal::CheckPrevOp(HalOperation expected) {
  if (prev_op_ != expected) {
    fprintf(stderr, "NativeHal: expected prev state %u, got %u\n",
            static_cast<uint8_t>(expected), static_cast<uint8_t>(prev_op_));
    assert(false);
  }
}

void NativeHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
  if (len == 0) {
    spi_printf("Warning: spiTransfer called with len=0\n");
  }

  if (len >= 2) {
    // Set status to OK
    in[0] = 0;
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
  } else if (out[0] == RADIOLIB_SX126X_CMD_WRITE_BUFFER) {
    spi_printf("SPI write buffer\n");
    packet_length_ = len - 2;
    assert(packet_length_ <= kPacketBufferSize);
    memcpy(packet_buffer_, &out[2], packet_length_);
    prev_op_ = HalOperation::kTransmitWrite;
  } else if (out[0] == RADIOLIB_SX126X_CMD_SET_TX) {
    CheckPrevOp(HalOperation::kTransmitWrite);
    prev_op_ = HalOperation::kTransmitTx;
  } else if (out[0] == RADIOLIB_SX126X_CMD_SET_RX) {
    prev_op_ = HalOperation::kReceiveRx;
  } else if (out[0] == RADIOLIB_SX126X_CMD_READ_BUFFER) {
    assert(len = packet_length_ + 3);
    CheckPrevOp(HalOperation::kReceiveRead);
    spi_printf("SPI buffer read, length %u\n", len);
    fflush(stdout);
    in[2] = 1;
    memcpy(&in[3], packet_buffer_, packet_length_);
    prev_op_ = HalOperation::kNone;
  } else if (out[0] == RADIOLIB_SX126X_CMD_GET_IRQ_STATUS) {
    assert(len == 4);
    in[2] = 0;
    in[3] = 0;
  } else if (out[0] == RADIOLIB_SX126X_CMD_GET_RX_BUFFER_STATUS) {
    spi_printf("SPI buffer length read: %u\n", packet_length_);
    CheckPrevOp(HalOperation::kReceiveRead);
    assert(len == 4);
    in[2] = packet_length_;
  } else if (out[0] == RADIOLIB_SX126X_CMD_SET_STANDBY) {
    spi_printf("SPI command: standby\n");
    assert(len == 2);
  } else {
    spi_printf("spiTransfer: ");
    for (size_t i = 0; i < len; i++) {
      spi_printf("%02X ", out[i]);
    }
    spi_printf("\n");
  }
}

bool NativeHal::TransmittedPacket() {
  return prev_op_ == HalOperation::kTransmitTx;
}

const uint8_t* NativeHal::GetTransmittedPacket(size_t* const len) {
  prev_op_ = HalOperation::kNone;
  *len = packet_length_;
  return packet_buffer_;
}

void NativeHal::SetReceivedPacket(const uint8_t* buffer, size_t len) {
  assert(len < kPacketBufferSize);

  CheckPrevOp(HalOperation::kReceiveRx);
  packet_length_ = len;
  memcpy(packet_buffer_, buffer, len);
  prev_op_ = HalOperation::kReceiveRead;
}