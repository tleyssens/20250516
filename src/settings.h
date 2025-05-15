#ifndef SETTINGS_H
#define SETTINGS_H

#include <WString.h> // Toegevoegd voor String-definitie
#include <EEPROM.h>
#include <NativeEthernet.h>


struct Storage {
  uint8_t Kp = 40;
  uint8_t lowPWM = 10;
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;
  float steerSensorCounts = 30;
  float AckermanFix = 1;
};
extern Storage steerSettings;

struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;
  uint8_t SteerButton = 0;
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;
};

struct ConfigIP {
  uint8_t ipOne = 192;
  uint8_t ipTwo = 168;
  uint8_t ipThree = 4;
};  ConfigIP networkAddress;   //3 bytes

extern Setup steerConfig;


void steerSettingsInit();
void steerConfigInit();
void settingsInit();

#endif