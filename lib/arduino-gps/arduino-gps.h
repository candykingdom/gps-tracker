#pragma once

#include <TinyGPSPlus.h>

#include "gps.h"

class ArduinoGps : public Gps {
 public:
  void Step() override;

  Coordinates GetCoordinates() override;

  bool Begin();

  // Prints the GPS output to the serial console.
  void StepDumpGpsOutput();

 private:
  TinyGPSPlus gps_;
};
