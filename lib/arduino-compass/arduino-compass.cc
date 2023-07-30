#include "arduino-compass.h"

float ArduinoCompass::GetHeadingRadians() {
  sensors_event_t event;
  compass_.getEvent(&event);
  int16_t x = compass_.raw.x + kCompassOffsetX;
  int16_t y = compass_.raw.y + kCompassOffsetY;
  return atan2(y, x);
}

bool ArduinoCompass::Begin() {
  if (!compass_.begin_SPI(kCompassCs)) {
    Serial2.println("Compass init failed");
    return false;
  }
  compass_.setDataRate(LIS2MDL_RATE_10_HZ);
  compass_.setOffsetCancellation(true);
  compass_.setLowPassFilter(true);

  return true;
}

void ArduinoCompass::DumpValuesForCalibration() {
  sensors_event_t event;
  compass_.getEvent(&event);
  Serial2.printf("Uni:0,0,0,0,0,0,%d,%d,%d\n", compass_.raw.x, compass_.raw.y,
                 compass_.raw.z);
}
