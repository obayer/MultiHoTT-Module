#ifndef MultiHoTTModule_h
#define MultiHoTTModule_h

struct {
  // 10mV steps
  uint16_t driveVoltage;

  uint16_t current;
  
  // 2mV steps
  uint8_t cell1;
  uint8_t cell2;
  uint8_t cell3;
  
  uint8_t temp;
 
  int32_t altitude;
} MultiHoTTModule;

struct {
  uint8_t version;
  uint8_t minDriveVoltage;
  uint8_t maxTemp1;
  int16_t maxAltitude;
  uint8_t varioBeep;
} MultiHoTTModuleSettings;

#endif
