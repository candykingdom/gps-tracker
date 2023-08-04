#include "RadioLib.h"
#include "types.h"

enum class HalOperation {
  kNone,
  kTransmitWrite,
  kTransmitTx,
  kReceiveRx,
  kReceiveRead,
};

class NativeHal : public RadioLibHal {
 public:
  static constexpr uint32_t kInput = 0;
  static constexpr uint32_t kOutput = 1;
  static constexpr uint32_t kRising = 2;
  static constexpr uint32_t kFalling = 3;

  NativeHal()
      : RadioLibHal(kInput, kOutput, /*low=*/0, /*high=*/1, kRising, kFalling) {
  }

  // implementations of pure virtual RadioLibHal methods
  void pinMode(uint32_t pin, uint32_t mode) override {}
  void digitalWrite(uint32_t pin, uint32_t value) override {}
  uint32_t digitalRead(uint32_t pin) override;
  void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void),
                       uint32_t mode) override {}
  void detachInterrupt(uint32_t interruptNum) override {}
  void delay(unsigned long ms) override {}
  void delayMicroseconds(unsigned long us) override {}
  unsigned long millis() override;
  unsigned long micros() override;
  long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
    return 0;
  }
  void spiBegin() override {}
  void spiBeginTransaction() override {}
  void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;
  void spiEndTransaction() override {}
  void spiEnd() override {}

  void readPersistentStorage(uint32_t addr, uint8_t* buff,
                             size_t len) override {}
  void writePersistentStorage(uint32_t addr, uint8_t* buff,
                              size_t len) override {}

  void setMillis(unsigned long millis);
  void setMicros(unsigned long micros);

  // For testing
  bool TransmittedPacket();
  const uint8_t* GetTransmittedPacket(size_t* len);
  void SetReceivedPacket(const uint8_t* buffer, size_t len);

 private:
  void CheckPrevOp(HalOperation expected);

  unsigned long millis_ = 0;
  unsigned long micros_ = 0;

  static constexpr size_t kPacketBufferSize = 200;
  uint8_t packet_buffer_[kPacketBufferSize] = {0};
  size_t packet_length_ = 0;
  HalOperation prev_op_ = HalOperation::kNone;
};
