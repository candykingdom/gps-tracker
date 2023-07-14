#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LTR329_LTR303.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>

#include <algorithm>

constexpr int kLed = PB4;

// SPI
constexpr int kMiso = PB2;
constexpr int kMosi = PA4;
constexpr int kSck = PB8;
constexpr uint8_t kCompassCs = PB9;

// I2C
constexpr int kScl = PA9;
constexpr int kSda = PA10;

Adafruit_LIS2MDL compass;
Adafruit_LTR303 light_sensor;
TinyGPSPlus gps;

void FatalError() {
  bool on = false;
  while (true) {
    digitalWrite(kLed, on);
    delay(100);
    on = !on;
  }
}

void setup() {
  // check if nBOOT_SEL bit is set
  if (FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) {
    // unlock flash/option
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F;

    while (FLASH->SR & FLASH_SR_BSY1)
      ;

    // clear nBOOT_SEL bit
    FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;

    // write
    FLASH->CR |= FLASH_CR_OPTSTRT;
    while (FLASH->SR & FLASH_SR_BSY1)
      ;
  }

  pinMode(kLed, OUTPUT);
  digitalWrite(kLed, HIGH);

  Serial2.begin(115200);
  // Serial2.printf("Booting...\n");

  SPI.setMISO(kMiso);
  SPI.setMOSI(kMosi);
  SPI.setSCLK(kSck);

  if (!compass.begin_SPI(kCompassCs)) {
    Serial2.println("Compass init failed");
    FatalError();
  }
  compass.setDataRate(LIS2MDL_RATE_10_HZ);
  compass.setOffsetCancellation(true);
  compass.setLowPassFilter(true);

  // TODO: re-enable when light sensor can be soldered reliably
  // Wire.setSCL(kScl);
  // Wire.setSDA(kSda);
  // Wire.begin();
  // Wire.setClock(100000);
  // if (!light_sensor.begin()) {
  //   Serial2.println("Light sensor init failed");
  //   FatalError();
  // }
  // light_sensor.setGain(LTR3XX_GAIN_1);
  // light_sensor.setIntegrationTime(LTR3XX_INTEGTIME_400);
  // light_sensor.setMeasurementRate(LTR3XX_MEASRATE_500);

  Serial3.begin(9600);
  delay(1000);
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

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  Serial2.println(heading);
}

// For use with the Jupyter notebook in Adafruit's calibration guide:
// https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-jupyter
// https://raw.githubusercontent.com/adafruit/Adafruit_SensorLab/master/notebooks/Mag_Gyro_Calibration.ipynb
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
  // TODO: read a response
}

void DumpGpsOutput() {
  while (Serial3.available()) {
    char in = Serial3.read();
    Serial2.print(in);
  }
}

void DumpGpsLocation() {
  while (Serial3.available()) {
    gps.encode(Serial3.read());
  }

  if (gps.satellites.isValid()) {
    Serial2.printf("Found %d satellites\n", gps.satellites.value());
  } else {
    Serial2.println("Found no satellites\n");
  }

  if (gps.location.isValid()) {
    Serial2.printf("Location: %3.3f, %3.3f\n", gps.location.lat(),
                   gps.location.lng());
  } else {
    Serial2.println("No valid GPS location");
  }

  Serial2.println();
}

uint32_t send_at = 0;

void loop() {
  // delay(100);
  // DumpCompassHeading();

  DumpGpsLocation();
  delay(1000);
}
