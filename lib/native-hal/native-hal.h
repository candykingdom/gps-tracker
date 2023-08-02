#include "RadioLib.h"
#include "types.h"

class NativeHal : public RadioLibHal {
 public:
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
  void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {}
  void spiEndTransaction() override {}
  void spiEnd() override {}

  void readPersistentStorage(uint32_t addr, uint8_t* buff,
                             size_t len) override {}
  void writePersistentStorage(uint32_t addr, uint8_t* buff,
                              size_t len) override {}

  void setMillis(unsigned long millis);
  void setMicros(unsigned long micros);

 private:
  unsigned long millis_ = 0;
  unsigned long micros_ = 0;
};
