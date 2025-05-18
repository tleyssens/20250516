#define SIMPLEFOC_DISABLE_TYPEDEF // Voorkomt naamconflict
#include <Arduino.h>
#include "gps.h"
#include "imu.h"
#include "motor.h"
#include "ethernet.h"
#include "can.h"
#include "settings.h"
#include "utils.h"


void init() {
  delay(500);
  Serial.begin(baudAOG);
  SimpleFOCDebug::enable();

  pinMode(GGAReceivedLED, OUTPUT);
  pinMode(Power_on_LED, OUTPUT);
  pinMode(Ethernet_Active_LED, OUTPUT);
  pinMode(GPSRED_LED, OUTPUT);
  pinMode(GPSGREEN_LED, OUTPUT);
  pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);

  gpsInit();
  imuInit();
  motorInit();
  ethernetInit();
  canInit();
  settingsInit();

  Serial.println("Initialization complete, waiting for GPS and AgOpenGPS");
}

void run() {
  while (true) {
    gpsRun();
    imuRun();
    motorRun();
    ethernetRun();
    canRun();

    if (Ethernet.linkStatus() == LinkOFF) {
      digitalWrite(Power_on_LED, HIGH);
      digitalWrite(Ethernet_Active_LED, LOW);
    } else if (Ethernet.linkStatus() == LinkON) {
      digitalWrite(Power_on_LED, LOW);
      digitalWrite(Ethernet_Active_LED, HIGH);
    }

    // Voorkom dat de lus te snel draait
    delay(1);
  }
}

int main() {
  init();
  run();
  return 0; // Wordt nooit bereikt
}
