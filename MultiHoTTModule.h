#ifndef MultiHoTTModule_h
#define MultiHoTTModule_h

struct {
  uint16_t vbat1;  //VBat MultiWiiModule
  uint16_t vbat2;  //VBat MultiWii FlightControll

  uint16_t cell1;
  uint16_t cell2;
  uint16_t cell3;
  uint16_t cell4;
  
  uint8_t temp;
  
  uint32_t altitude;
  uint16_t current;
} MultiHoTTModule;

struct {
  uint8_t alarmVBat;
  uint8_t alarmTemp1;
} MultiHoTTModuleSettings;

#endif
