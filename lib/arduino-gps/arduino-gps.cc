#include "arduino-gps.h"

void ArduinoGps::Step() {
  while (Serial3.available() > 0) {
    char in = Serial3.read();
    gps_.encode(in);
  }
}

Coordinates ArduinoGps::GetCoordinates() {
  if (!gps_.location.isValid()) {
    return Coordinates();
  }

  return Coordinates(gps_.location.lat(), gps_.location.lng());
}

bool ArduinoGps::Begin() {
  Serial3.begin(9600);

  // TODO: increase baud? Default is 9600, which is pretty slow.

  // Disable all output messages except for position. We don't use the other
  // messages, and the serial input buffer is small.

  // Disable most output messages, except for GGA (which contains position)
  // Format:
  // $PMTK314,<GLL>,<RMC>,<VTG>,<GGA>,<GSA>,<GSV>,<Res1>,<Res2>,<Res3>,<Res4>,
  // <Res5>,<Res6>,<Res7>,<Res8>,<Res9>,<Res10>,<Res11>,<Res12>,<Res13>,<Res14>,
  // <GBS>,<Res16>
  Serial3.println(
      AppendChecksum("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"));

  // Disable TXT messages.
  // Format: $PQTXT,W,<Mode>,<Save>
  Serial3.println(AppendChecksum("$PQTXT,W,0,0"));

  // TODO: enter GLP (adaptive low-power) mode

  // TODO: check that we recieve some sort of response from the GPS

  return true;
}

void ArduinoGps::StepDumpGpsOutput() {
  while (Serial3.available()) {
    char in = Serial3.read();
    Serial2.print(in);
  }
}
