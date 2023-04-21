#include <Adafruit_LIS2MDL.h>
#include <Arduino.h>

#include <algorithm>

constexpr int kLed = PB4;
constexpr int kMiso = PB2;
constexpr int kMosi = PA4;
constexpr int kSck = PB8;
constexpr uint8_t kCompassCs = PB9;

Adafruit_LIS2MDL compass;

void FatalError() {
  bool on = false;
  while (true) {
    digitalWrite(kLed, on);
    delay(100);
    on = !on;
  }
}

void setup() {
  // check if nBOOT_SEL bit is set
  if (FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) {
    // unlock flash/option
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F;

    while (FLASH->SR & FLASH_SR_BSY1)
      ;

    // clear nBOOT_SEL bit
    FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;

    // write
    FLASH->CR |= FLASH_CR_OPTSTRT;
    while (FLASH->SR & FLASH_SR_BSY1)
      ;
  }

  pinMode(kLed, OUTPUT);
  digitalWrite(kLed, HIGH);

  Serial2.begin(115200);
  Serial2.printf("Booting...\n");

  SPI.setMISO(kMiso);
  SPI.setMOSI(kMosi);
  SPI.setSCLK(kSck);

  if (!compass.begin_SPI(kCompassCs)) {
    Serial2.printf("Compass init failed");
    FatalError();
  }
  compass.setDataRate(LIS2MDL_RATE_10_HZ);
  compass.setOffsetCancellation(true);
  compass.setLowPassFilter(true);
}

void DumpCompassMinMax() {
  static int16_t x_min = 1000;
  static int16_t x_max = 0;
  static int16_t y_min = 1000;
  static int16_t y_max = 0;
  static int16_t z_min = 1000;
  static int16_t z_max = 0;

  sensors_event_t event;
  compass.getEvent(&event);

  x_min = std::min(compass.raw.x, x_min);
  x_max = std::max(compass.raw.x, x_max);

  y_min = std::min(compass.raw.y, y_min);
  y_max = std::max(compass.raw.y, y_max);

  z_min = std::min(compass.raw.z, z_min);
  z_max = std::max(compass.raw.z, z_max);
  Serial2.printf("%6d, %6d, %6d, %6d, %6d, %6d\n", x_min, x_max, y_min, y_max,
                 z_min, z_max);
}

void DumpCompassHeading() {
  sensors_event_t event;
  compass.getEvent(&event);
  int16_t x = compass.raw.x + 527;
  int16_t y = compass.raw.y + 435;
  int16_t z = compass.raw.z + 167;
  float heading = (atan2(y, x) * 180) / 3.141593;
  float heading = -1 * (atan2(event.magnetic.x,event.magnetic.y) * 180)
  / 3.141593;

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  // Serial2.println(heading);
  // Serial2.printf("x:%4d, y:%4d, z:%4d\n", x, y, z);
}

void loop() {
  DumpCompassHeading();
  delay(200);
}
