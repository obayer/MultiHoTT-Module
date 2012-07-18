#ifndef HoTTv4_h
#define HoTTv4_h

/** ###### HoTT module specifications ###### */

#define HOTTV4_GENERAL_AIR_SENSOR_ID 0xD0

#define HOTTV4_ELECTRICAL_AIR_SENSOR_ID 0x8E // Electric Air Sensor ID
#define HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID 0xE0 // Electric Air Module ID

const uint8_t kHoTTv4BinaryPacketSize = 45; 
const uint8_t kHoTTv4TextPacketSize = 173;

typedef enum {
  HoTTv4NotificationErrorCalibration     = 0x01,
  HoTTv4NotificationErrorReceiver        = 0x02,
  HoTTv4NotificationErrorDataBus         = 0x03,
  HoTTv4NotificationErrorNavigation      = 0x04,
  HoTTv4NotificationErrorError           = 0x05,
  HoTTv4NotificationErrorCompass         = 0x06,
  HoTTv4NotificationErrorSensor          = 0x07,
  HoTTv4NotificationErrorGPS             = 0x08,
  HoTTv4NotificationErrorMotor           = 0x09,
  
  HoTTv4NotificationMaxTemperature       = 0x0A,
  HoTTv4NotificationAltitudeReached      = 0x0B,
  HoTTv4NotificationWaypointReached      = 0x0C,
  HoTTv4NotificationNextWaypoint         = 0x0D,
  HoTTv4NotificationLanding              = 0x0E,
  HoTTv4NotificationGPSFix               = 0x0F,
  HoTTv4NotificationUndervoltage         = 0x10,
  HoTTv4NotificationGPSHold              = 0x11,
  HoTTv4NotificationGPSHome              = 0x12,
  HoTTv4NotificationGPSOff               = 0x13,
  HoTTv4NotificationBeep                 = 0x14,
  HoTTv4NotificationMicrocopter          = 0x15,
  HoTTv4NotificationCapacity             = 0x16,
  HoTTv4NotificationCareFreeOff          = 0x17,
  HoTTv4NotificationCalibrating          = 0x18,
  HoTTv4NotificationMaxRange             = 0x19,
  HoTTv4NotificationMaxAltitude          = 0x1A,
  
  HoTTv4Notification20Meter              = 0x25,
  HoTTv4NotificationMicrocopterOff       = 0x26,
  HoTTv4NotificationAltitudeOn           = 0x27,
  HoTTv4NotificationAltitudeOff          = 0x28,
  HoTTv4Notification100Meter             = 0x29,
  HoTTv4NotificationCareFreeOn           = 0x2E,
  HoTTv4NotificationDown                 = 0x2F,
  HoTTv4NotificationUp                   = 0x30,
  HoTTv4NotificationHold                 = 0x31,
  HoTTv4NotificationGPSOn                = 0x32,
  HoTTv4NotificationFollowing            = 0x33,
  HoTTv4NotificationStarting             = 0x34,
  HoTTv4NotificationReceiver             = 0x35,
} HoTTv4Notification;

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
} HoTTV4ElectricAirModule;

struct {
  uint8_t startByte;
  uint8_t sensorTextID;
  uint8_t alarm;
  uint8_t text[8*21];
  uint8_t endByte;
  uint8_t chksum;
} HoTTv4ElectricalAirTextModule;

#endif
