#include "can.h"
#include "utils.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

void canInit() {
  Can0.begin();
  Can0.setBaudRate(250000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canReceive);
}

void canRun() {
  Can0.events();
}

void canReceive(const CAN_message_t &msg) {
  // Placeholder voor CAN-ontvangstlogica
}