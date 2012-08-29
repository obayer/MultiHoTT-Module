#include "HoTTv4.h"
#include "MultiHoTTModule.h"

#define HOTTV4_RXTX 6
#define HOTTV4_TX_DELAY 1000

#define HOTTV4_BUTTON_DEC 0xEB
#define HOTTV4_BUTTON_INC 0xED
#define HOTTV4_BUTTON_SET 0xE9
#define HOTTV4_BUTTON_NIL 0x0F
#define HOTTV4_BUTTON_NEXT 0xEE
#define HOTTV4_BUTTON_PREV 0xE7

#define OFFSET_HEIGHT 500
#define OFFSET_M2S 72
#define OFFSET_M3S 120

static uint8_t outBuffer[173];

static uint8_t row = 2;
static uint8_t col = 0;

static uint8_t vbat = 100;

SoftwareSerial hottV4Serial(HOTTV4_RXTX , HOTTV4_RXTX);

/** 
 * Common setup method for HoTTv4
 */
void hottV4Setup() {
  hottV4Serial.begin(19200);

  hottV4EnableReceiverMode();
}

/**
 * Enables RX and disables TX
 */
static inline void hottV4EnableReceiverMode() {
  DDRD &= ~(1 << HOTTV4_RXTX);
  PORTD |= (1 << HOTTV4_RXTX);
}

/**
 * Enabels TX and disables RX
 */
static inline void hottV4EnableTransmitterMode() {
  DDRD |= (1 << HOTTV4_RXTX);
}

/**
 * Writes out given byte to HoTT serial interface.
 * If in debug mode, data is also written to UART serial interface. 
 */
static void hottV4SerialWrite(uint8_t c) {
  hottV4Serial.write(c);
}

/**
 * Writes cell 1-4 high, low values and if not available 
 * calculates vbat.
 */
static void hottV4EAMUpdateBattery() {
  HoTTV4ElectricAirModule.cell1L = MultiHoTTModule.cell1/2;
  if (HoTTV4ElectricAirModule.cell1H < MultiHoTTModule.cell1/2) {
    HoTTV4ElectricAirModule.cell1H = MultiHoTTModule.cell1/2;
  }
  HoTTV4ElectricAirModule.cell2L = MultiHoTTModule.cell2/2;
  if (HoTTV4ElectricAirModule.cell2H < MultiHoTTModule.cell2/2) {
    HoTTV4ElectricAirModule.cell2H = MultiHoTTModule.cell2/2;
  }
  HoTTV4ElectricAirModule.cell3L = MultiHoTTModule.cell3/2;
  if (HoTTV4ElectricAirModule.cell3H < MultiHoTTModule.cell3/2) {
    HoTTV4ElectricAirModule.cell3H = MultiHoTTModule.cell3/2;
  }
  HoTTV4ElectricAirModule.cell4L = MultiHoTTModule.cell4/2;
  if (HoTTV4ElectricAirModule.cell4H < MultiHoTTModule.cell4/2) {
    HoTTV4ElectricAirModule.cell4H = MultiHoTTModule.cell4/2;
  }
  HoTTV4ElectricAirModule.driveVoltageLow = MultiHoTTModule.vbat1 & 0xFF;
  HoTTV4ElectricAirModule.driveVoltageHigh = MultiHoTTModule.vbat1 >> 8;
  HoTTV4ElectricAirModule.battery1Low = MultiHoTTModule.vbat1 & 0xFF; 
  HoTTV4ElectricAirModule.battery1High = MultiHoTTModule.vbat1 >> 8; 
  
  #ifdef MultiWii_VBat
    HoTTV4ElectricAirModule.battery2Low = MultiHoTTModule.vbat2 & 0xFF; 
    HoTTV4ElectricAirModule.battery2High = MultiHoTTModule.vbat2 >> 8;
  #endif

  if ( MultiHoTTModule.vbat1 <= MultiHoTTModuleSettings.alarmVBat) {
    HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4ElectricAirModule.alarmInverse1 |= 0x80; // Invert Voltage display
  } 
}

static void hottV4EAMUpdateTemperatures() {
  HoTTV4ElectricAirModule.temp1 = 20 + MultiHoTTModule.temp;
  HoTTV4ElectricAirModule.temp2 = 20;

  //if (HoTTV4ElectricAirModule.temp1 >= (20 + MultiHoTTModuleSettings.alarmTemp1)) {
  //  HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationMaxTemperature;  
  //  HoTTV4ElectricAirModule.alarmInverse |= 0x8; // Invert Temp1 display
  //}
}

static void hottV4GPSUpdate() {
  //number of Satelites
  HoTTV4GPSModule.GPSNumSat=MultiHoTTModule.GPS_numSat;
  if (MultiHoTTModule.GPS_fix) { 
    /** GPS fix */
    HoTTV4GPSModule.GPS_fix = 0x66; // Displays a 'f' for fix
    //latitude
    HoTTV4GPSModule.LatitudeNS=(MultiHoTTModule.GPS_latitude<0);
    uint8_t deg = MultiHoTTModule.GPS_latitude / 100000;
    uint32_t sec = (MultiHoTTModule.GPS_latitude - (deg * 100000)) * 6;
    uint8_t min = sec / 10000;
    sec = sec % 10000;
    uint16_t degMin = (deg * 100) + min;
    HoTTV4GPSModule.LatitudeMinLow = degMin;
    HoTTV4GPSModule.LatitudeMinHigh = degMin >> 8; 
    HoTTV4GPSModule.LatitudeSecLow = sec; 
    HoTTV4GPSModule.LatitudeSecHigh = sec >> 8;
    //latitude
    HoTTV4GPSModule.longitudeEW=(MultiHoTTModule.GPS_longitude<0);
    deg = MultiHoTTModule.GPS_longitude / 100000;
    sec = (MultiHoTTModule.GPS_longitude - (deg * 100000)) * 6;
    min = sec / 10000;
    sec = sec % 10000;
    degMin = (deg * 100) + min;
    HoTTV4GPSModule.longitudeMinLow = degMin;
    HoTTV4GPSModule.longitudeMinHigh = degMin >> 8; 
    HoTTV4GPSModule.longitudeSecLow = sec; 
    HoTTV4GPSModule.longitudeSecHigh = sec >> 8;
    /** GPS Speed in km/h */
    uint16_t speed = (MultiHoTTModule.GPS_speed / 100) * 36; // 0.1m/s * 0.36 = km/h
    HoTTV4GPSModule.GPSSpeedLow = speed & 0x00FF;
    HoTTV4GPSModule.GPSSpeedHigh = speed >> 8;
    /** Distance to home */
    HoTTV4GPSModule.distanceLow = MultiHoTTModule.GPS_distanceToHome & 0x00FF;
    HoTTV4GPSModule.distanceHigh = MultiHoTTModule.GPS_distanceToHome >> 8; 
    /** Altitude */
    HoTTV4GPSModule.altitudeLow = MultiHoTTModule.GPS_altitude & 0x00FF;
    HoTTV4GPSModule.altitudeHigh = MultiHoTTModule.GPS_altitude >> 8;
    /** Altitude */
    HoTTV4GPSModule.HomeDirection = MultiHoTTModule.GPS_directionToHome;
    
  } else {
    HoTTV4GPSModule.GPS_fix = 0x20; // Displays a ' ' to show nothing or clear the old value
  }
}


/**
 * Sends HoTTv4 capable EAM telemetry frame.
 */
static void hottV4SendEAM() {
  /** Minimum data set for EAM */
  HoTTV4ElectricAirModule.startByte = 0x7C;
  HoTTV4ElectricAirModule.sensorID = HOTTV4_ELECTRICAL_AIR_SENSOR_ID;
  HoTTV4ElectricAirModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTV4ElectricAirModule.endByte = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4ElectricAirModule.alarmTone = 0x0;
  HoTTV4ElectricAirModule.alarmInverse1 = 0x0;
  
  hottV4EAMUpdateBattery();
  hottV4EAMUpdateTemperatures();

  HoTTV4ElectricAirModule.current = MultiHoTTModule.current / 10; 
  HoTTV4ElectricAirModule.height = OFFSET_HEIGHT + MultiHoTTModule.altitude;
  HoTTV4ElectricAirModule.m2s = OFFSET_M2S; 
  HoTTV4ElectricAirModule.m3s = OFFSET_M3S;
  
  #ifdef DEBUG_HOTT
      LCD_set_line(3);
      LCD_Print("EAM ");
      print_VBAT(MultiHoTTModule.vbat1);
      LCD_Print(" ");
      print_VBAT(MultiHoTTModule.vbat2);
//    Serial.println(" --- EAM --- ");
//    Serial.print("     Low: ");
//    Serial.print(HoTTV4ElectricAirModule.battery1Low);
//    Serial.print("    High: ");
//    Serial.println(HoTTV4ElectricAirModule.battery1High);
//    Serial.print("   VBat1: ");
//    Serial.println(HoTTV4ElectricAirModule.driveVoltageLow + (HoTTV4ElectricAirModule.driveVoltageHigh * 0x100), DEC);
//    Serial.print("   VBat2: ");
//    Serial.println(HoTTV4ElectricAirModule.battery2Low + (HoTTV4ElectricAirModule.battery2High * 0x100), DEC);
//    Serial.print("Current: ");
//    Serial.println(HoTTV4ElectricAirModule.current, DEC);
//    Serial.println("");
  #endif

  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/**
 * Sends HoTTv4 capable GPS telemetry frame.
 */
static void hottV4SendGPS() {
  /** Minimum data set for EAM */
  HoTTV4GPSModule.startByte = 0x7C;
  HoTTV4GPSModule.sensorID = HOTTV4_GPS_SENSOR_ID;
  HoTTV4GPSModule.sensorTextID = HOTTV4_GPS_SENSOR_TEXT_ID;
  HoTTV4GPSModule.endByte = 0x7D;
  /** ### */
 
  /** Reset alarms */
  HoTTV4GPSModule.alarmTone = 0x0;
  HoTTV4GPSModule.alarmInverse1 = 0x0;
  
  hottV4GPSUpdate();
 
  #ifdef DEBUG_HOTT
      LCD_set_line(4);
      print_GPSLine1(MultiHoTTModule.GPS_numSat);
      LCD_set_line(5);
      print_GPSLine2(MultiHoTTModule.GPS_longitude,MultiHoTTModule.GPS_latitude);
  #endif

  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/* ##################################################################### *
 *                HoTTv4 Text Mode                                       *
 * ##################################################################### */

static void hottV4ClearAllTextLines() {
  memset(&HoTTv4ElectricalAirTextModule.text[0], ' ', 8*21);
}

/**
 * Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule
 */
static void hottV4WriteLine(uint8_t line, const char *text) {
  uint8_t writeText = 1;

  for (uint8_t index = 0; index < 21; index++) {
    if (0x0 != text[index] && writeText) {
       HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = text[index];
    } else {
      writeText = 0;
      HoTTv4ElectricalAirTextModule.text[(line * 21) + index] = ' ';
    }
  }
}

/**
 * Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule.
 * If row == line it gets a selection indicator and given row is also highlighted.
 */
static void hottV4WriteLine(uint8_t line, const char *text, uint8_t row, uint8_t col) {
  char lineText[21];
  uint8_t inCol = 0;

  enum {
    IDLE,
    COLON,
    SPACE,
    COL,
    DONE,
  } state = IDLE;

  const char selectionIndicator = (line == row) ? '>' : ' ';  
  snprintf(lineText, 21, "%c%s", selectionIndicator, text);  
  
  for (uint8_t index = 0 ; index < 21 ; index++) {
    uint8_t c = lineText[index];
    
    if (IDLE == state) {
      state = (':' == c) ? COLON : IDLE;
    } else if (COLON == state) {
      state = (' ' == c) ? SPACE : COLON; 
    } else if (SPACE == state) {
      if ('.' <= c) {
        inCol++;
        state = COL;
      } else {
        state = SPACE;
      }
    } else if (COL == state) {
      if (' ' == c) {
        state = SPACE;
      } else if (0x0 == c) {
        state = DONE;
      } else {
        state = COL;
      }
    } else if (DONE == c) {
      break;
    }
    
    if ((COL == state) && (inCol == col) && (line == row)) {
      lineText[index] += 128;
    } 
  }

  hottV4WriteLine(line, lineText);
}

/**
 * Sends HoTTv4 capable EAM text frame.
 */
static void hottV4SendEAMText(uint8_t row, uint8_t col) {
  /** Minimum data set for EAM Text mode */
  HoTTv4ElectricalAirTextModule.startByte = 0x7B;
  HoTTv4ElectricalAirTextModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTv4ElectricalAirTextModule.endByte = 0x7D;

  // Clear output buffer
  memset(&outBuffer, 0x0, sizeof(outBuffer));
  
  hottV4ClearAllTextLines();
  hottV4WriteLine(0, " MultiHoTT Settings");

  char text[21];
  snprintf(text, 21, "ALARM VOLT : %2i.%1iV", MultiHoTTModuleSettings.alarmVBat / 10, MultiHoTTModuleSettings.alarmVBat % 10);
  hottV4WriteLine(2, text, row, col);
  
  snprintf(text, 21, "ALARM TEMP1:  %3iC", MultiHoTTModuleSettings.alarmTemp1);
  hottV4WriteLine(3, text, row, col);
 
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTv4ElectricalAirTextModule, kHoTTv4TextPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4TextPacketSize);
}

/**
 * Expects an array of at least size bytes. All bytes till size will be transmitted
 * to the HoTT capable receiver. Last byte will always be treated as checksum and is
 * calculated on the fly.
 */
static void hottV4SendData(uint8_t *data, uint8_t size) {
  hottV4Serial.flush();
  
  // Protocoll specific waiting time
  // to avoid collisions
  delay(5);
  
  if (hottV4Serial.available() == 0) {
    hottV4EnableTransmitterMode();
    
    uint16_t crc = 0;

    for (uint8_t i = 0; i < (size - 1); i++) {
      crc += data[i];     
      hottV4SerialWrite(data[i]);
      
      // Protocoll specific delay between each transmitted byte
      delayMicroseconds(HOTTV4_TX_DELAY);
    }
    
    // Write package checksum
    hottV4SerialWrite(crc & 0xFF);
    
    hottV4EnableReceiverMode();  
  }
}

/**
 * Entry point to send HoTTv4 capable data frames according to the
 * requested module.
 */
void hottV4SendTelemetry() {
  static enum _hottV4_state {
    IDLE,
    BINARY,
    TEXT,
  } hottV4_state = IDLE;
  
  if (hottV4Serial.available() > 1) {
    //Serial.println(" --- HOTT --- ");
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t c = hottV4Serial.read();

      if (IDLE == hottV4_state) {
        switch (c) {
          case 0x80:
            hottV4_state = BINARY;
            break;
          case 0x7F:
            hottV4_state = TEXT;
            break;
          default:
            hottV4_state = IDLE;
        }
      } else if (BINARY == hottV4_state) {
        switch (c) {
          case HOTTV4_ELECTRICAL_AIR_SENSOR_ID:
            //Serial.println(" --- EAM --- ");
            hottV4SendEAM();
            hottV4_state = IDLE;
            break;
          case HOTTV4_GPS_SENSOR_ID:
            //Serial.println(" --- GPS --- ");
            hottV4SendGPS();
            hottV4_state = IDLE;
            break;
          default:
            hottV4_state = IDLE;
        }
      } else if (TEXT == hottV4_state) {
        switch (c) {
          case HOTTV4_BUTTON_NEXT:
            break;
          case HOTTV4_BUTTON_PREV:
            break;
          case HOTTV4_BUTTON_DEC:
            if (col) {
              if (row == 2) {
                MultiHoTTModuleSettings.alarmVBat -= 1;
              } else if (row == 3) {
                MultiHoTTModuleSettings.alarmTemp1 -= 1;
              }
            } else {
              row = row > 2 ? row - 1 : row;
            }
            break;
          case HOTTV4_BUTTON_INC:
            if (col) {
              if (row == 2) {
                MultiHoTTModuleSettings.alarmVBat += 1;
              } else if (row == 3) {
                MultiHoTTModuleSettings.alarmTemp1 += 1;
              }
            } else {
              row = row < 3 ? row + 1 : row;
            }
            break;
          case HOTTV4_BUTTON_SET:
            col = col == 1 ? 0 : 1;
            break;
        }
        
        hottV4SendEAMText(row, col);
        hottV4_state = IDLE;
      }
    }
  }
}
