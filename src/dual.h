#ifndef DUAL_H
#define DUAL_H

#include <Arduino.h>
#include <NativeEthernet.h>

void relPosDecode(uint8_t* ackPacket); // Geef ackPacket als parameter
void imuHandler();

#endif