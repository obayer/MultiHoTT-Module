#ifndef HoTTv4_h
#define HoTTv4_h

/** ###### HoTT module specifications ###### */

#define HOTTV4_GENERAL_AIR_SENSOR_ID 0xD0

#define HOTTV4_ELECTRICAL_AIR_SENSOR_ID 0x8E // Electric Air Sensor ID
#define HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID 0xE0 // Electric Air Module ID

const uint8_t kHoTTv4BinaryPacketSize = 45; 

struct {
  uint8_t startByte;
  uint8_t sensorID;
  uint8_t alarmTone; /* Alarm */
  uint8_t sensorTextID;
  uint16_t alarmInverse;

  uint8_t cell1L; /* Low Voltage Cell 1-7 in 2mV steps */
  uint8_t cell2L;
  uint8_t cell3L;
  uint8_t cell4L;
  uint8_t cell5L;
  uint8_t cell6L;
  uint8_t cell7L;
  uint8_t cell1H;   /* High Voltage Cell 1-7 in 2mV steps */
  uint8_t cell2H;  
  uint8_t cell3H;  
  uint8_t cell4H;
  uint8_t cell5H;
  uint8_t cell6H;
  uint8_t cell7H;

  uint16_t battery1; /* Battetry 1 LSB/MSB in 100mv steps; 50 == 5V */
  uint16_t battery2; /* Battetry 2 LSB/MSB in 100mv steps; 50 == 5V */

  uint8_t temp1; /* Temp 1; Offset of 20. 20 == 0C */
  uint8_t temp2; /* Temp 2; Offset of 20. 20 == 0C */

  uint16_t height; /* Height. Offset -500. 500 == 0 */
  uint16_t current; /* 1 = 0.1A */
  uint16_t driveVoltage;
  uint16_t capacity; /* mAh */
  uint16_t m2s; /* m2s; 0x48 == 0 */
  uint8_t m3s; /* m3s; 0x78 == 0 */
  
  uint16_t rpm; /* RPM. 10er steps; 300 == 3000rpm */
  uint8_t minutes;
  uint8_t seconds;
  uint8_t speed;

  uint8_t version;
  uint8_t endByte;
  uint8_t chksum;
} HottV4ElectricAirModule;

#endif
