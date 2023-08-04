#include "radio.h"

#include <gtest/gtest.h>

#include <cstdio>

extern void SetRadioIdle();

TEST(RadioTest, Initializes) {
  Radio radio;
  ASSERT_TRUE(radio.Begin());
}

TEST(RadioTest, Transmits) {
  Radio radio;
  ASSERT_TRUE(radio.Begin());
  EXPECT_TRUE(radio.IsIdle());

  char str[6] = "hello";
  EXPECT_EQ(radio.StartTransmit((uint8_t*)str, 5), 0);
  EXPECT_FALSE(radio.IsIdle());

  NativeHal& hal = radio.GetHal();
  ASSERT_TRUE(hal.TransmittedPacket());
  size_t received_length = 0;
  EXPECT_STREQ((char*)hal.GetTransmittedPacket(&received_length), str);
  EXPECT_EQ(received_length, 5);

  radio.Step();
  EXPECT_FALSE(radio.IsIdle());

  SetRadioIdle();
  EXPECT_FALSE(radio.IsIdle());
  radio.Step();
  EXPECT_TRUE(radio.IsIdle());
}

TEST(RadioTest, TransmitFailsWhenNotIdle) {
  Radio radio;
  ASSERT_TRUE(radio.Begin());
  ASSERT_TRUE(radio.IsIdle());

  char str[6] = "hello";
  ASSERT_EQ(radio.StartTransmit((uint8_t*)str, 5), 0);
  ASSERT_FALSE(radio.IsIdle());

  ASSERT_EQ(radio.StartTransmit((uint8_t*)str, 5), RADIOLIB_ERR_TX_TIMEOUT);
}
