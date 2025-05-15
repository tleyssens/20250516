#ifndef CAN_H
#define CAN_H

#include <FlexCAN_T4.h>
#include <NativeEthernet.h>

void canInit();
void canRun();
void canReceive(const CAN_message_t &msg);

#endif