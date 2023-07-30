#pragma once

struct Coordinates {
  explicit Coordinates() : valid(false), latitude(0), longitude(0) {}

  explicit Coordinates(double latitude, double longitude)
      : valid(true), latitude(latitude), longitude(longitude) {}

  const bool valid;
  const double latitude;
  const double longitude;
};

class Gps {
 public:
  virtual void Step() {}

  virtual Coordinates GetCoordinates() = 0;

  // Calculates the NMEA checksum and appends it
  static char* AppendChecksum(const char* const sentence);
};
