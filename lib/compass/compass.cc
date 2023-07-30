#include "compass.h"

float Compass::GetHeadingDegrees() {
  return RadiansToDegrees(GetHeadingRadians());
}

float Compass::RadiansToDegrees(float radians) {
  float heading = (radians * 180.0) / kPi;

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }

  return heading;
}