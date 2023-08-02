#include <Adafruit_GFX.h>  // Core graphics library
#include <Adafruit_LTR329_LTR303.h>
#include <Adafruit_ST7789.h>  // Hardware-specific library for ST7789
#include <Arduino.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>

#include <algorithm>

#include "arduino-compass.h"
#include "arduino-gps.h"
#include "radio.h"

constexpr bool kUseScreen = false;

constexpr int kLed = PB4;

// SPI
constexpr int kMiso = PB2;
constexpr int kMosi = PA4;
constexpr int kSck = PB8;

// Radio
constexpr int kRadioCs = PA15;
constexpr int kRadioDio1 = PB6;
constexpr int kRadioBusy = PB7;
constexpr int kRadioRxen = PC6;

// Screen
constexpr int kScreenDc = PA11;
constexpr int kScreenCs = PA12;
constexpr int kScreenBlk = PA7;
constexpr int kScreenReset = PA8;
Adafruit_ST7789 screen = Adafruit_ST7789(kScreenCs, kScreenDc, kScreenReset);

// I2C
constexpr int kScl = PA9;
constexpr int kSda = PA10;

Adafruit_LTR303 light_sensor;
ArduinoCompass compass;
ArduinoGps gps;

SPISettings radio_spi_settings(10 * 1000 * 1000, MSBFIRST, SPI_MODE0);
SX1262 radio = new Module(kRadioCs, kRadioDio1, /*rst=*/RADIOLIB_NC,
                          /*gpio=*/kRadioBusy, SPI, radio_spi_settings);
volatile bool radio_idle = true;

void SetRadioIdle(void) {
  radio_idle = true;
  // digitalWrite(kLed, LOW);
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

void ConfigureScreen() {
  screen.init(/*width=*/240, /*height=*/320);
  screen.setSPISpeed(20 * 1000 * 1000);
  screen.fillScreen(ST77XX_BLACK);
  screen.setTextSize(2);
  screen.setTextWrap(false);
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
  pinMode(kScreenBlk, OUTPUT);
  digitalWrite(kScreenBlk, HIGH);

  pinMode(kRadioRxen, OUTPUT);
  digitalWrite(kRadioRxen, HIGH);

  Serial2.begin(115200);
  Serial2.printf("Booting...\n");

  SPI.setMISO(kMiso);
  SPI.setMOSI(kMosi);
  SPI.setSCLK(kSck);

  delay(100);
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

  if (!compass.Begin()) {
    Serial2.println("Failed to initialize compass");
    FatalError();
  }

  if (!gps.Begin()) {
    Serial2.println("Failed to initialize GPS");
    FatalError();
  }

  // Configure radio
  Serial2.print("Initializing radio... ");
  int state = radio.begin(/*freq=*/915.0, /*bw*/ 125, /*sf=*/9, /*cr=*/7,
                          /*syncWord=*/18, /*power=*/22, /*preambleLength=*/8,
                          /*txcoVoltage=*/1.8, /*useRegulatorLdo=*/false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial2.println("success.");
  } else {
    Serial2.print("failed, code ");
    Serial2.println(state);
    FatalError();
  }
  radio.setDio1Action(SetRadioIdle);
  radio.setRfSwitchPins(kRadioRxen, RADIOLIB_NC);
  state = radio.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    Serial2.printf("`radio.setDio2AsRfSwitch` failed: %d\n");
  }
  radio.startReceive();

  // digitalWrite(kLed, LOW);

  if (kUseScreen) {
    ConfigureScreen();
  }
}

void DisplayCompass() {
  static constexpr float kCircleRadius = 100;
  static constexpr int16_t kCompassCenterX = 120;
  static constexpr int16_t kCompassCenterY = 160;
  static int16_t compass_x = 0;
  static int16_t compass_y = 0;
  static float heading = 0;

  screen.drawLine(kCompassCenterX, kCompassCenterY, compass_x, compass_y,
                  ST77XX_BLACK);

  heading = compass.GetHeadingRadians();
  compass_x = kCompassCenterX + kCircleRadius * cos(2 * 3.141593 - heading);
  compass_y = kCompassCenterY + kCircleRadius * sin(2 * 3.141593 - heading);
  screen.drawLine(kCompassCenterX, kCompassCenterY, compass_x, compass_y,
                  ST77XX_WHITE);

  screen.fillRect(/*x=*/0, /*y=*/0, /*w=*/50, /*h=*/15, ST77XX_BLACK);
  screen.setCursor(0, 0);
  screen.setTextColor(ST77XX_WHITE);
  screen.printf("%3d", (int16_t)Compass::RadiansToDegrees(heading));
}

void DumpLightSensor() {
  bool valid;
  uint16_t visible_plus_ir, infrared;

  if (light_sensor.newDataAvailable()) {
    valid = light_sensor.readBothChannels(visible_plus_ir, infrared);
    if (valid) {
      Serial2.print("CH0 Visible + IR: ");
      Serial2.print(visible_plus_ir);
      Serial2.print("\t\tCH1 Infrared: ");
      Serial2.println(infrared);
    } else {
      Serial2.println("Light sensor data invalid");
    }
  }
}

uint32_t print_at = 0;
constexpr uint32_t kPrintEvery = 1000;

uint32_t screen_update_at = 0;
constexpr uint32_t kScreenUpdateEvery = 500;

uint32_t received_packet_at = 0;
uint32_t force_send_packet_at = 4000;
int16_t last_rssi = 0;
int16_t last_snr = 0;

void Transmit() {
  Serial2.printf("Transmitting... (%d)\n", radio_idle);
  // digitalWrite(kRadioRxen, LOW);
  // delay(1);
  int state = radio.startTransmit("test message");
  if (state == RADIOLIB_ERR_NONE) {
    Serial2.println("Radio start transmit success!");
  } else {
    Serial2.printf("Radio transmit failed, code: %d\n", state);
  }
  // digitalWrite(kRadioRxen, HIGH);
  // delay(1);
}

constexpr float kLatitude = 0;
constexpr float kLongitude = 0;

void loop() {
  static bool last_op_transmit = false;

  // int state = radio.startReceive();
  // if (state == RADIOLIB_ERR_NONE) {
  //   Serial2.println("Radio receive success!");
  // } else {
  //   Serial2.print("Radio receive failed, code ");
  //   Serial2.println(state);
  //   delay(500);
  // }

  gps.Step();

  if (millis() > 3000 && radio_idle) {
    radio_idle = false;
    if (last_op_transmit) {
      Serial2.println("Starting receive");
      last_op_transmit = false;
      radio.startReceive();
    } else {
      // We received a packet
      received_packet_at = millis();
      String received;
      int state = radio.readData(received);
      if (state == RADIOLIB_ERR_NONE) {
        Serial2.print("*** Received: ");
        Serial2.println(received);
        digitalWrite(kLed, HIGH);
        delay(10);
        digitalWrite(kLed, LOW);
        delay(10);
      }
      last_rssi = radio.getRSSI();
      last_snr = radio.getSNR();
      radio.standby();
      last_op_transmit = true;
      Transmit();
    }
  }

  if (!last_op_transmit && millis() > force_send_packet_at) {
    force_send_packet_at = millis() + 500 + random(100);
    radio.standby();
    last_op_transmit = true;
    Transmit();
  }

  if (millis() > print_at) {
    Serial2.println(millis());
    print_at = millis() + kPrintEvery;
  }

  digitalWrite(kLed, (millis() / 100) % 25 == 0);

  if (kUseScreen && millis() > screen_update_at) {
    // DisplayCompass();

    screen.fillRect(/*x=*/0, /*y=*/0, /*w=*/120, /*h=*/64, ST77XX_BLACK);
    screen.setCursor(0, 0);
    screen.setTextColor(ST77XX_WHITE);
    screen.printf("Up: %6u\n", millis() - received_packet_at);
    screen.printf("RSSI: %4d\n", last_rssi);
    screen.printf("SNR:  %4d\n", last_snr);
    Coordinates coordinates = gps.GetCoordinates();
    if (coordinates.valid) {
      screen.printf("Dis: %5d", (int32_t)TinyGPSPlus::distanceBetween(
                                    coordinates.latitude, coordinates.longitude,
                                    kLatitude, kLongitude));
    }

    screen_update_at = millis() + kScreenUpdateEvery;
  }
}
