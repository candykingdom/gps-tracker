#pragma once

#include <Adafruit_LIS2MDL.h>

#include "compass.h"

class ArduinoCompass : public Compass {
 public:
  float GetHeadingRadians() override;

  bool Begin();

  void DumpValuesForCalibration();

 private:
  Adafruit_LIS2MDL compass_;

  static constexpr int kCompassCs = PB9;

  static constexpr int16_t kCompassOffsetX = 173;
  static constexpr int16_t kCompassOffsetY = 210;
  static constexpr int16_t kCompassOffsetZ = -89;
};
