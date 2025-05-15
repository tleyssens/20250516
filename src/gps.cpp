#include <Arduino.h>
#include "gps.h"
#include "zNMEAParser.h"
#include "utils.h"

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;
bool GGA_Available = false;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;
uint32_t gpsReadyTime = 0;
bool blink = false;

// USB voor AgIO

HardwareSerial* SerialGPS = &Serial7; // Serial7 op pins 28/29 op Teensy 4.1 Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)
HardwareSerial* SerialRTK = &Serial3; // Serial3 op pins 0/1)
#ifdef ARDUINO_TEENSY41
usb_serial_class* SerialAOG = &Serial;  // USB voor AgIO op Teensy
#endif
//HardwareSerial* SerialAOG = &Serial;  // USB voor AgIO


char nmea[100] = "";
char fixTime[12] = "";
char latitude[15] = "";
char latNS[3] = "";
char longitude[15] = "";
char lonEW[3] = "";
char fixQuality[2] = "";
char numSats[4] = "";
char HDOP[5] = "";
char altitude[12] = "";
char ageDGPS[10] = "";
char speedKnots[10] = "";
char imuHeading[6] = "";
char imuRoll[6] = "";
char imuPitch[6] = "";
char imuYawRate[6] = "";
bool passThroughGPS = false;
bool passThroughGPS2 = false;
EthernetUDP Eth_udpPAOGI;

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];
uint8_t GPStxbuffer[serial_buffer_size];
uint8_t GPS2rxbuffer[serial_buffer_size];
uint8_t GPS2txbuffer[serial_buffer_size];
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer


uint32_t PortSwapTime = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int relposnedByteCount = 0;

void CalculateChecksum() {
  int16_t sum = 0;
  char tmp;
  for (int inx = 1; inx < 200; inx++) {
    tmp = nmea[inx];
    if (tmp == '*') break;
    sum ^= tmp;
  }

  byte chk = (sum >> 4);
  char hex[2] = { "0123456789ABCDEF"[chk], 0 };
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = { "0123456789ABCDEF"[chk], 0 };
  strcat(nmea, hex2);
}

void relPosDecode()
{
  dualReadyRelPos = true;
  // Placeholder voor RELPOSNED-decodering
}

void gpsInit()
{
  SerialGPS->begin(baudGPS);
  Serial.println("Start setup");

  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  SerialRTK->begin(baudRTK);
  SerialRTK->addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  SerialGPS2->begin(baudGPS);
  SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
  SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

  parser.setErrorHandler(errorHandler);
  parser.addHandler("GPGGA", GGA_Handler);
  parser.addHandler("GPVTG", VTG_Handler);
}

void gpsRun()
{
  HardwareSerial *SerialGPSTmp = nullptr;

  if (!GGA_Available && !passThroughGPS && !passThroughGPS2)
  {
    if (systick_millis_count - PortSwapTime >= 10000)
    {
      SerialGPSTmp = SerialGPS;
      SerialGPS = SerialGPS2;
      SerialGPS2 = SerialGPSTmp;
      PortSwapTime = systick_millis_count;
    }
  }

  if (SerialAOG->available())
  {
    uint8_t incoming_char = SerialAOG->read();
    if (passThroughGPS)
    {
      SerialGPS->write(incoming_char);
    }
    else if (passThroughGPS2)
    {
      SerialGPS2->write(incoming_char);
    }
    else
    {
      SerialGPS->write(incoming_char);
    }
  }

  if (SerialGPS->available())
  {
    if (passThroughGPS)
    {
      SerialAOG->write(SerialGPS->read());
    }
    else
    {
      parser << SerialGPS->read();
    }
  }

  if (SerialGPS2->available())
  {
    uint8_t incoming_char = SerialGPS2->read();
    if (passThroughGPS2)
    {
      SerialAOG->write(incoming_char);
    }
    else
    {
      if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
      {
        relposnedByteCount++;
      }
      else if (relposnedByteCount > 3)
      {
        ackPacket[relposnedByteCount] = incoming_char;
        relposnedByteCount++;
      }
      else
      {
        relposnedByteCount = 0;
      }
    }
  }

  if (relposnedByteCount > 71)
  {
    if (calcChecksum())
    {
      digitalWrite(GPSRED_LED, LOW);
      useDual = true;
      relPosDecode();
    }
    relposnedByteCount = 0;
  }

  if (dualReadyGGA && dualReadyRelPos)
  {
    BuildNmea();
    dualReadyGGA = false;
    dualReadyRelPos = false;
  }

  if ((systick_millis_count - gpsReadyTime) > 10000)
  {
    digitalWrite(GPSRED_LED, LOW);
    digitalWrite(GPSGREEN_LED, LOW);
    useDual = false;
  }
}

void GGA_Handler()
{
  parser.getArg(0, fixTime);
  parser.getArg(1, latitude);
  parser.getArg(2, latNS);
  parser.getArg(3, longitude);
  parser.getArg(4, lonEW);
  parser.getArg(5, fixQuality);
  parser.getArg(6, numSats);
  parser.getArg(7, HDOP);
  parser.getArg(8, altitude);
  parser.getArg(12, ageDGPS);

  digitalWrite(GGAReceivedLED, blink ? HIGH : LOW);
  blink = !blink;

  GGA_Available = true;
  if (useDual) {
    dualReadyGGA = true;
  }
  
  if (useBNO08x || useCMPS) {
    imuHandler();
    BuildNmea();
    dualReadyGGA = false;
    if (!useDual) {
      digitalWrite(GPSRED_LED, HIGH);
      digitalWrite(GPSGREEN_LED, LOW);
    }
  } else if (!useBNO08x && !useCMPS && !useDual) {
    digitalWrite(GPSRED_LED, blink);
    digitalWrite(GPSGREEN_LED, LOW);
    itoa(65535, imuHeading, 10);
    BuildNmea();
  }
  gpsReadyTime = systick_millis_count;
}

void VTG_Handler() {
  float speedKnots = 0.0f; // Initialiseren om waarschuwing te vermijden
  parser.getArg(4, speedKnots); // Snelheid in knopen (veld 4)
  gpsSpeed = speedKnots * 1.852 / 3.6; // Omzetten van knopen naar m/s
}

void BuildNmea()
{
  strcpy(nmea, "");
  strcat(nmea, useDual ? "$PAOGI," : "$PANDA,");
  strcat(nmea, fixTime);
  strcat(nmea, ",");
  strcat(nmea, latitude);
  strcat(nmea, ",");
  strcat(nmea, latNS);
  strcat(nmea, ",");
  strcat(nmea, longitude);
  strcat(nmea, ",");
  strcat(nmea, lonEW);
  strcat(nmea, ",");
  strcat(nmea, fixQuality);
  strcat(nmea, ",");
  strcat(nmea, numSats);
  strcat(nmea, ",");
  strcat(nmea, HDOP);
  strcat(nmea, ",");
  strcat(nmea, altitude);
  strcat(nmea, ",");
  strcat(nmea, ageDGPS);
  strcat(nmea, ",");
  strcat(nmea, speedKnots);
  strcat(nmea, ",");
  strcat(nmea, imuHeading);
  strcat(nmea, ",");
  strcat(nmea, imuRoll);
  strcat(nmea, ",");
  strcat(nmea, imuPitch);
  strcat(nmea, ",");
  strcat(nmea, imuYawRate);
  strcat(nmea, "*");

  CalculateChecksum();
  strcat(nmea, "\r\n");

  if (!passThroughGPS && !passThroughGPS2) {
    SerialAOG->write(nmea);
  }

  if (Ethernet_running) {
    int len = strlen(nmea);
    Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
    Eth_udpPAOGI.write(nmea, len);
    Eth_udpPAOGI.endPacket();
  }
}

