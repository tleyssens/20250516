#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <NativeEthernet.h>
#include "zNMEAParser.h"
#include "imu.h"

extern HardwareSerial* SerialGPS; // Voor RTK GPS
extern HardwareSerial* SerialRTK;
extern usb_serial_class*  SerialAOG; // Voor AgOpenGPS-communicatie


extern char nmea[100];
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];
extern char speedKnots[10];
extern char imuHeading[6];
extern char imuRoll[6];
extern char imuPitch[6];
extern char imuYawRate[6];
extern bool passThroughGPS;
extern bool passThroughGPS2;
extern bool dualReadyGGA;
extern bool dualReadyRelPos;
extern bool GGA_Available;
extern uint32_t PortSwapTime;
extern uint32_t gpsReadyTime;
extern EthernetUDP Eth_udpPAOGI;



// extern NMEAParser<2> parser;
// extern bool GGA_Available;
// extern uint32_t PortSwapTime;
// extern uint32_t gpsReadyTime;

void gpsInit();
void gpsRun();
void GGA_Handler();
void VTG_Handler();
void BuildNmea();
void relPosDecode();

#endif