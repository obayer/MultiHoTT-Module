#ifndef MultiHoTTModule_h
#define MultiHoTTModule_h

struct {
  uint16_t vbat1;  //VBat MultiWiiModule
  uint8_t vbat2;  //VBat MultiWii FlightControll
  uint16_t intPowerMeterSum; 
  uint16_t rssi;
  uint16_t amperage;
  
  uint16_t cell1;
  uint16_t cell2;
  uint16_t cell3;
  uint16_t cell4;
  
  uint8_t temp;
  
  uint16_t attitudeAngles1;
  uint16_t attitudeAngles2;
  uint16_t attitudeHeading;
  
  uint32_t altitude;
  uint16_t current;

  uint8_t GPS_fix;
  uint8_t GPS_numSat;
  uint32_t GPS_latitude;
  uint32_t GPS_longitude;
  uint16_t GPS_altitude;
  uint16_t GPS_speed;

  //MSP_COMP_GPS
  uint16_t GPS_distanceToHome;
  uint16_t GPS_directionToHome;
  uint8_t GPS_update;    

  //MSP_HEADING
  uint8_t magMode;
  uint16_t heading;
  uint16_t magHold;
  uint16_t headFreeModeHold;
  
} MultiHoTTModule;

struct {
  uint8_t alarmVBat;
  uint8_t alarmTemp1;
} MultiHoTTModuleSettings;

#endif
