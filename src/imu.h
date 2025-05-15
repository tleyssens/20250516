#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <NativeEthernet.h>
#include "BNO08x_AOG.h"

#define ImuWire Wire
extern bool useCMPS;
extern bool useBNO08x;
extern bool invertRoll;
extern float correctionHeading;
extern BNO080 bno08x;

extern uint32_t READ_BNO_TIME;
#define CMPS14_ADDRESS 0x60

void imuInit();
void imuRun();
void readBNO();
void imuHandler();

#endif