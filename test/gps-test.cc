#include "gps.h"

#include <gtest/gtest.h>

TEST(GpsTest, AppendChecksum) {
  EXPECT_STREQ(Gps::AppendChecksum(
                   "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"),
               "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35");

  EXPECT_STREQ(Gps::AppendChecksum("$PQTXT,W,0,0"), "$PQTXT,W,0,0*22");
}
