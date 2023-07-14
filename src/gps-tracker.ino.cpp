# 1 "/tmp/tmp5qo9ueai"
#include <Arduino.h>
# 1 "/home/adam/hardware/gps-tracker/src/gps-tracker.ino"
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LTR329_LTR303.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>

#include <algorithm>

constexpr int kLed = PB4;


constexpr int kMiso = PB2;
constexpr int kMosi = PA4;
constexpr int kSck = PB8;
constexpr uint8_t kCompassCs = PB9;


constexpr int kScl = PA9;
constexpr int kSda = PA10;

Adafruit_LIS2MDL compass;
Adafruit_LTR303 light_sensor;
TinyGPSPlus gps;
void FatalError();
void setup();
void DumpCompassMinMax();
void DumpCompassHeading();
void DumpCompassValuesForCalibration();
void DumpLightSensor();
void GetGpsFirmwareVersion();
void DumpGpsOutput();
void DumpGpsLocation();
void loop();
#line 24 "/home/adam/hardware/gps-tracker/src/gps-tracker.ino"
void FatalError() {
  bool on = false;
  while (true) {
    digitalWrite(kLed, on);
    delay(100);
    on = !on;
  }
}

void setup() {

  if (FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) {

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F;

    while (FLASH->SR & FLASH_SR_BSY1)
      ;


    FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;


    FLASH->CR |= FLASH_CR_OPTSTRT;
    while (FLASH->SR & FLASH_SR_BSY1)
      ;
  }

  pinMode(kLed, OUTPUT);
  digitalWrite(kLed, HIGH);

  Serial2.begin(115200);


  SPI.setMISO(kMiso);
  SPI.setMOSI(kMosi);
  SPI.setSCLK(kSck);

  delay(100);
  if (!compass.begin_SPI(kCompassCs)) {
    Serial2.println("Compass init failed");
    FatalError();
  }
  compass.setDataRate(LIS2MDL_RATE_10_HZ);
  compass.setOffsetCancellation(true);
  compass.setLowPassFilter(true);
# 86 "/home/adam/hardware/gps-tracker/src/gps-tracker.ino"
  Serial3.begin(9600);
}

void DumpCompassMinMax() {
  static int16_t x_min = 1000;
  static int16_t x_max = 0;
  static int16_t y_min = 1000;
  static int16_t y_max = 0;
  static int16_t z_min = 1000;
  static int16_t z_max = 0;

  sensors_event_t event;
  compass.getEvent(&event);

  x_min = std::min(compass.raw.x, x_min);
  x_max = std::max(compass.raw.x, x_max);

  y_min = std::min(compass.raw.y, y_min);
  y_max = std::max(compass.raw.y, y_max);

  z_min = std::min(compass.raw.z, z_min);
  z_max = std::max(compass.raw.z, z_max);
  Serial2.printf("%6d, %6d, %6d, %6d, %6d, %6d\n", x_min, x_max, y_min, y_max,
                 z_min, z_max);
}

void DumpCompassHeading() {
  sensors_event_t event;
  compass.getEvent(&event);
  int16_t x = compass.raw.x - 147 + 297;
  int16_t y = compass.raw.y - 76.5 + 151.0;
  int16_t z = compass.raw.z + 6 - 19.5;
  float heading = (atan2(y, x) * 180) / 3.141593;


  if (heading < 0) {
    heading = 360 + heading;
  }
  Serial2.println(heading);
}




void DumpCompassValuesForCalibration() {
  sensors_event_t event;
  compass.getEvent(&event);
  Serial2.printf("Uni:0,0,0,0,0,0,%d,%d,%d\n", compass.raw.x, compass.raw.y,
                 compass.raw.z);
}

void DumpLightSensor() {
  bool valid;
  uint16_t visible_plus_ir, infrared;

  if (light_sensor.newDataAvailable()) {
    valid = light_sensor.readBothChannels(visible_plus_ir, infrared);
    if (valid) {
      Serial.print("CH0 Visible + IR: ");
      Serial.print(visible_plus_ir);
      Serial.print("\t\tCH1 Infrared: ");
      Serial.println(infrared);
    } else {
      Serial.println("Light sensor data invalid");
    }
  }
}

void GetGpsFirmwareVersion() {
  Serial3.println("$PQVERNO,R*3F");

}

void DumpGpsOutput() {
  while (Serial3.available()) {
    char in = Serial3.read();
    Serial2.print(in);
  }
}

void DumpGpsLocation() {







  while (Serial3.available() > 0) {
    gps.encode(Serial3.read());
  }

  Serial2.printf(
      "GPS comms:  with_fix=%d, chars=%d, checksum_failed=%d, "
      "checksum_passed=%d\n",
      gps.sentencesWithFix(), gps.charsProcessed(), gps.failedChecksum(),
      gps.passedChecksum());

  if (gps.satellites.isValid()) {
    Serial2.printf("Found %d satellites\n", gps.satellites.value());
  } else {
    Serial2.println("Found no satellites");
  }

  if (gps.location.isValid()) {
    Serial2.print("Location: ");
    Serial2.print(gps.location.lat(), 3);
    Serial2.print(", ");
    Serial2.println(gps.location.lng(), 3);
  } else {
    Serial2.println("No valid GPS location");
  }

  Serial2.println();
}

uint32_t send_at = 0;

void loop() {



  DumpGpsLocation();

  delay(1000);
}