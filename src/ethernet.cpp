#include <ethernet.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "settings.h" // Toegevoegd voor steerConfig
#include "utils.h"


uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];
uint8_t Eth_NTRIP_packetBuffer[UDP_TX_PACKET_MAX_SIZE];

EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpNtrip;
EthernetUDP Eth_udpAutoSteer;
IPAddress Eth_ipDestination(192, 168, 1, 255);

void ethernetInit() {

  Serial.println("Starting Ethernet...");
  Ethernet.begin(mac, Eth_myip);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Ethernet_running = false;
    digitalWrite(Ethernet_Active_LED, LOW);
  } else if (Ethernet.linkStatus() == LinkOFF) {
    Ethernet_running = false;
    digitalWrite(Ethernet_Active_LED, LOW);
  } else {
    Ethernet_running = true;
    digitalWrite(Ethernet_Active_LED, HIGH);
    Eth_udpAutoSteer.begin(AOGAutoSteerPort); //Start UDP op poort 8888
    Eth_udpNtrip.begin(AOGNtripPort);
    Eth_udpPAOGI.begin(portMy); // Voor PAOGI
  }
  IPAddress Eth_ipDestination(192, 168, 1, 255);
}



void ReceiveUdp() {
  if (!Ethernet_running) return;

  uint16_t len = Eth_udpAutoSteer.parsePacket();
  if (len > 4) {
    Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

    if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) {
      if (autoSteerUdpData[3] == 0xFE) {
        gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
        gpsSpeedUpdateTimer = 0;

        prevGuidanceStatus = guidanceStatus;
        guidanceStatus = autoSteerUdpData[7];
        guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

        steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01;

        if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1)) {
          watchdogTimer = WATCHDOG_FORCE_VALUE;
        } else {
          watchdogTimer = 0;
        }

        tram = autoSteerUdpData[10];
        relay = autoSteerUdpData[11];
        relayHi = autoSteerUdpData[12];

        int16_t sa = (int16_t)(steerAngleActual * 100);
        PGN_253[5] = (uint8_t)sa;
        PGN_253[6] = sa >> 8;
        PGN_253[7] = switchByte;
        PGN_253[8] = (uint8_t)pwmDisplay;

        int16_t CK_A = 0;
        for (uint8_t i = 2; i < PGN_253_Size; i++) {
          CK_A += PGN_253[i];
        }
        PGN_253[PGN_253_Size] = CK_A;

        SendUdp(PGN_253, PGN_253_Size + 1, Eth_ipDestination, portDestination);

        if (steerConfig.PressureSensor || steerConfig.CurrentSensor) {
          if (aog2Count++ > 2) {
            PGN_250[5] = (byte)sensorReading;
            CK_A = 0;
            for (uint8_t i = 2; i < PGN_250_Size; i++) {
              CK_A += PGN_250[i];
            }
            PGN_250[PGN_250_Size] = CK_A;
            SendUdp(PGN_250, PGN_250_Size + 1, Eth_ipDestination, portDestination);
            aog2Count = 0;
          }
        }
      }
    }
  }
}

void udpNtrip() {
  if (!Ethernet_running) return;

  unsigned int packetLength = Eth_udpNtrip.parsePacket();
  if (packetLength > 0) {
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
    SerialGPS->write(Eth_NTRIP_packetBuffer, packetLength);
  }
}

void ethernetRun() {
  ReceiveUdp();
  udpNtrip();
}

void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport) {
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}