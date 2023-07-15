#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LTR329_LTR303.h>
#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>

#include <algorithm>

constexpr int kLed = PB4;

// SPI
constexpr int kMiso = PB2;
constexpr int kMosi = PA4;
constexpr int kSck = PB8;
constexpr int kCompassCs = PB9;

constexpr int kRadioCs = PA15;
constexpr int kRadioDio1 = PB6;
constexpr int kRadioBusy = PB7;
constexpr int kRadioRxen = PC6;

// I2C
constexpr int kScl = PA9;
constexpr int kSda = PA10;

Adafruit_LIS2MDL compass;
Adafruit_LTR303 light_sensor;
TinyGPSPlus gps;

SPISettings radio_spi_settings(10 * 1000 * 1000, MSBFIRST, SPI_MODE0);
SX1262 radio = new Module(kRadioCs, kRadioDio1, /*rst=*/RADIOLIB_NC,
                          /*gpio=*/kRadioBusy, SPI, radio_spi_settings);
volatile bool radio_idle = true;

void SetRadioIdle(void) {
  radio_idle = true;
  digitalWrite(kLed, HIGH);
}

void FatalError() {
  bool on = false;
  while (true) {
    digitalWrite(kLed, on);
    delay(100);
    on = !on;
  }
}

// http://www.hhhh.org/wiml/proj/nmeaxor.html
String withChecksum(String sentence) {
  bool started = false;
  char checksum = 0;
  for (uint32_t index = 0; index < sentence.length(); index++) {
    if (index > 0 && sentence[index - 1] == '$') {
      checksum = sentence[index];
      started = true;
      continue;  // Skip the rest of this loop iteration.
    }

    if (sentence[index] == '*') {
      break;  // Exit the loop.
    }

    // Ignore everything preceeding '$'.
    if (!started) {
      continue;  // Skip the rest of this loop iteration.
    }

    checksum = checksum xor sentence[index];
  }

  String sentenceWithChecksum =
      sentence + (checksum < 10 ? "0" : "") + String(checksum, HEX);
  return sentenceWithChecksum;
}

// Sets up the GPS.
void ConfigureGps() {
  // TODO: increase baud? Default is 9600, which is pretty slow.

  // Disable all output messages except for position. We don't use the other
  // messages, and the serial input buffer is small.

  // Disable most output messages, except for GGA (which contains position)
  // Format:
  // $PMTK314,<GLL>,<RMC>,<VTG>,<GGA>,<GSA>,<GSV>,<Res1>,<Res2>,<Res3>,<Res4>,
  // <Res5>,<Res6>,<Res7>,<Res8>,<Res9>,<Res10>,<Res11>,<Res12>,<Res13>,<Res14>,
  // <GBS>,<Res16>
  Serial3.println(
      withChecksum("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"));

  // Disable TXT messages.
  // Format: $PQTXT,W,<Mode>,<Save>
  Serial3.println(withChecksum("$PQTXT,W,0,0*"));

  // TODO: enter GLP (adaptive low-power) mode
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
  // digitalWrite(kLed, HIGH);

  // pinMode(kRadioBusy, INPUT);

  Serial2.begin(115200);
  Serial2.printf("Booting...\n");

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

  // Configure GPS
  Serial3.begin(9600);
  ConfigureGps();

  // Configure radio
  Serial2.print("Initializing radio... ");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial2.println("success.");
  } else {
    Serial2.print("failed, code ");
    Serial2.println(state);
    FatalError();
  }
  radio.setDio1Action(SetRadioIdle);
  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    Serial2.printf("`radio.setDio2AsRfSwitch` failed: %d\n");
  }

  state = radio.setFrequency(915.0);
  if (state != RADIOLIB_ERR_NONE) {
    Serial2.printf("`radio.setFrequency` failed: %d\n");
  }

  radio.setOutputPower(0);

  // digitalWrite(kLed, LOW);
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
  static const char* location = R"str(
$GNGGA,010809.000,4002.299834,N,10515.657245,W,2,12,0.84,1612.078,M,-20.609,M,,*7B
)str";
  // for (char const* c = location; *c != 0; c++) {
  //   gps.encode(*c);
  // }

  static uint32_t print_at = 0;
  static constexpr uint32_t kPrintEvery = 1000;

  while (Serial3.available() > 0) {
    char in = Serial3.read();
    gps.encode(in);
  }

  if (millis() < print_at) {
    return;
  }
  print_at = millis() + kPrintEvery;

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

uint32_t print_at = 0;
constexpr uint32_t kPrintEvery = 1000;

void loop() {
  // DumpGpsLocation();
  // DumpGpsOutput();
  // delay(1000);
  // int state = radio.startReceive();
  // if (state == RADIOLIB_ERR_NONE) {
  //   Serial2.println("Radio receive success!");
  // } else {
  //   Serial2.print("Radio receive failed, code ");
  //   Serial2.println(state);
  //   delay(500);
  // }
  if (radio_idle || millis() > print_at) {
    Serial2.printf("Transmitting... (%d)\n", radio_idle);
    radio_idle = false;
    // static constexpr uint32_t kSize = 8;
    // static const char message[kSize] = "12345678";
    // int state = radio.transmit("test message");
    int state = radio.startTransmit("test message");
    Serial2.printf("Radio status: %u\n", radio.getStatus());
    if (state == RADIOLIB_ERR_NONE) {
      Serial2.println("Radio transmit success!");
    } else {
      Serial2.printf("Radio transmit failed, code: %d\n", state);
    }
    digitalWrite(kLed, HIGH);
    delay(10);
    digitalWrite(kLed, LOW);
    delay(10);
    Serial2.printf("Radio status: %u\n", radio.getStatus());
  }
  // radio.startReceive();

  if (millis() > print_at) {
    Serial2.println(millis());
    print_at = millis() + kPrintEvery;
  }

  delay(10);
}
