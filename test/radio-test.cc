#include "radio.h"

#include <gtest/gtest.h>

#include <cstdio>

TEST(RadioTest, Initializes) {
  Radio radio;
  ASSERT_TRUE(radio.Begin());
}
