#include <avr/eeprom.h>
#include "MultiHoTTModule.h"

#define EEPROM_SETTINGS_VERSION 1 

void readSettings() {
  eeprom_read_block((void*)&MultiHoTTModuleSettings, (void*)0, sizeof(MultiHoTTModuleSettings));
}

void writeSettings() {
  eeprom_write_block((const void*)&MultiHoTTModuleSettings, (void*)0, sizeof(MultiHoTTModuleSettings));
}

void checkSettings() {
  if (MultiHoTTModuleSettings.version == EEPROM_SETTINGS_VERSION) {
    return;
  } else {
    MultiHoTTModuleSettings.version = EEPROM_SETTINGS_VERSION;
    MultiHoTTModuleSettings.alarmDriveVoltage = 104;
    MultiHoTTModuleSettings.alarmTemp1 = 50;

    writeSettings();
  }
}
