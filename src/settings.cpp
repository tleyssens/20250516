#include "settings.h"
#include "utils.h"

Storage steerSettings;
Setup steerConfig;
//IPAddress networkAddress(192, 168, 1, 121); //welk IP ?

#define EEP_Ident 2400
#define LOW_HIGH_DEGREES 3.0

void steerSettingsInit() {
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void steerConfigInit() {
  if (steerConfig.CytronDriver) {
    pinMode(PWM2_RPWM, OUTPUT);
  }
}

void settingsInit() {
  int16_t EEread;
  EEPROM.get(0, EEread);
  if (EEread != EEP_Ident) {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.put(60, networkAddress);
  } else {
    EEPROM.get(10, steerSettings);
    EEPROM.get(40, steerConfig);
    EEPROM.get(60, networkAddress);
  }

  steerSettingsInit();
  steerConfigInit();
}