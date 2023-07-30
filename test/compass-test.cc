#include <gtest/gtest.h>

#include "compass.h"

TEST(CompassTest, RadiansToDegrees) {
  EXPECT_FLOAT_EQ(Compass::RadiansToDegrees(0), 0);
  EXPECT_FLOAT_EQ(Compass::RadiansToDegrees(Compass::kPi / 2.0), 90);
  EXPECT_FLOAT_EQ(Compass::RadiansToDegrees(Compass::kPi), 180);
  EXPECT_FLOAT_EQ(Compass::RadiansToDegrees(Compass::kPi * 4.0), 0);
  EXPECT_FLOAT_EQ(Compass::RadiansToDegrees(Compass::kPi * 9.0 / 2.0), 90);

  EXPECT_FLOAT_EQ(Compass::RadiansToDegrees(-Compass::kPi / 2.0), 270);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // if you plan to use GMock, replace the line above with
  // ::testing::InitGoogleMock(&argc, argv);

  if (RUN_ALL_TESTS())
    ;

  // Always return zero-code and allow PlatformIO to parse results
  return 0;
}
