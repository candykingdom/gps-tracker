#include "gps.h"

#include <cstdio>

#include "types.h"

// static
char* Gps::AppendChecksum(const char* const sentence) {
  constexpr uint32_t kBufferSize = 200;
  constexpr uint32_t kChecksumLength = 4;
  static char buffer[kBufferSize];

  bool started = false;
  uint8_t checksum = 0;
  uint32_t i = 0;
  for (; sentence[i] != 0; i++) {
    if (i > (kBufferSize - kChecksumLength)) {
#ifdef ARDUINO
      Serial2.println("NMEA sentence too long for buffer in AppendChecksum");
      return buffer;
#else   // ARDUINO
      assert((i < kBufferSize,
              "NMEA sentence too long for buffer in AppendChecksum"));
#endif  // ARDUINO
    }

    if (i > 0 && sentence[i - 1] == '$') {
      checksum = sentence[i];
      started = true;
      continue;  // Skip the rest of this loop iteration.
    }

    // Ignore everything preceeding '$'.
    if (!started) {
      continue;  // Skip the rest of this loop iteration.
    }

    checksum ^= sentence[i];
  }

  snprintf(buffer, kBufferSize, "%s*%02X", sentence, checksum);

  return buffer;
}