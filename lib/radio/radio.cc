#include "radio.h"

// Not a class member because `setDio1Action` takes a C-style function
volatile bool dio_rose = false;

void SetRadioIdle() { dio_rose = true; }

bool Radio::Begin() {
  Serial2.print("Initializing radio... ");
  int state = radio_.begin(/*freq=*/915.0, /*bw*/ 125, /*sf=*/9, /*cr=*/7,
                           /*syncWord=*/18, /*power=*/22, /*preambleLength=*/8,
                           /*txcoVoltage=*/1.8, /*useRegulatorLdo=*/false);
  if (state == RADIOLIB_ERR_NONE) {
    Serial2.println("success.");
  } else {
    Serial2.printf("failed, code: %d\n", state);
    return false;
  }

  radio_.setDio1Action(SetRadioIdle);
  radio_.setRfSwitchPins(kRadioRxen, RADIOLIB_NC);
  state = radio_.setDio2AsRfSwitch(true);
  if (state != RADIOLIB_ERR_NONE) {
    Serial2.printf("`radio_.setDio2AsRfSwitch` failed: %d\n", state);
    return false;
  }

  // TODO: start receiving?

  return true;
}

void Radio::Step() {
  if (dio_rose) {
    if (prev_op_ == RadioOperation::kTransmit) {
      radio_.finishTransmit();
      prev_op_ = RadioOperation::kNone;
    } else if (prev_op_ == RadioOperation::kReceive) {
      packet_length_ = radio_.getPacketLength();
      int16_t read_status = radio_.readData(packet_buffer_, packet_length_);
      if (read_status != RADIOLIB_ERR_NONE) {
        packet_length_ = 0;
      }
      prev_op_ = RadioOperation::kNone;
    }
    dio_rose = false;
  }
}

bool Radio::IsIdle() { return prev_op_ == RadioOperation::kNone; }

int16_t Radio::StartTransmit(uint8_t* const data, const size_t len) {
  if (!IsIdle()) {
    Serial2.println("Error: tried to transmit while not idle");
    return RADIOLIB_ERR_TX_TIMEOUT;
  }

  prev_op_ = RadioOperation::kTransmit;
  dio_rose = false;
  return radio_.startTransmit(data, len);
}

int16_t Radio::ReceivedPacketLength() { return packet_length_; }

uint8_t* Radio::GetPacketBuffer() { return packet_buffer_; }

int16_t Radio::StartReceive() {
  if (!IsIdle()) {
    Serial2.println("Error: tried to receive while not idle");
    return RADIOLIB_ERR_RX_TIMEOUT;
  }

  prev_op_ = RadioOperation::kReceive;
  dio_rose = false;
  return radio_.startReceive();
}

int16_t Radio::Standby() {
  // TODO: should this be allowed if the radio is currently transmitting?
  prev_op_ = RadioOperation::kNone;
  dio_rose = false;
  return radio_.standby();
}

#ifndef ARDUINO
NativeHal& Radio::GetHal() { return hal_; }
#endif  // ARDUINO
