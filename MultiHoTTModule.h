#ifndef MultiHoTTModule_h
#define MultiHoTTModule_h

struct {
  uint8_t vbat;

  uint8_t cell1;
  uint8_t cell2;
  uint8_t cell3;
  
  uint8_t temp;
  
  uint32_t height;
  uint16_t current;
} MultiHoTTModule;

struct {
  uint8_t alarmVBat;
  uint8_t alarmTemp1;
} MultiHoTTModuleSettings;

#endif
