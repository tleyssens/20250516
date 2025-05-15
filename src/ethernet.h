#ifndef ETHERNET_H
#define ETHERNET_H

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

extern EthernetUDP Eth_udpAutoSteer;  // Globale UDP-instantie
extern bool Ethernet_running;
extern uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];
extern uint8_t Eth_NTRIP_packetBuffer[UDP_TX_PACKET_MAX_SIZE];
extern IPAddress Eth_ipDestination;
extern unsigned int portDestination;

void ethernetInit();
void ethernetRun();
void ReceiveUdp();
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport);
void udpNtrip();

#endif