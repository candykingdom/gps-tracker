#pragma once

class Compass {
 public:
  virtual float GetHeadingRadians() = 0;

  float GetHeadingDegrees();

  static float RadiansToDegrees(float radians);

  static constexpr float kPi = 3.14159265;
};
