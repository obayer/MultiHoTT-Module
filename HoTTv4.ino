#include "HoTTv4.h"
#include "MultiHoTTModule.h"

#define HOTTV4_RXTX 3 
#define HOTTV4_TX_DELAY 1000

#define HOTTV4_BUTTON_DEC 0xEB
#define HOTTV4_BUTTON_INC 0xED
#define HOTTV4_BUTTON_SET 0xE9
#define HOTTV4_BUTTON_NIL 0x0F
#define HOTTV4_BUTTON_NEXT 0xEE
#define HOTTV4_BUTTON_PREV 0xE7

#define OFFSET_ALTITUDE 500
#define OFFSET_M2S 72
#define OFFSET_M3S 120

static uint8_t outBuffer[173];

static uint8_t row = 2;
static uint8_t col = 0;

SoftwareSerial hottV4Serial(HOTTV4_RXTX , HOTTV4_RXTX);

/* ##################################################################### *
 *                HoTTv4 Common communication                            *
 * ##################################################################### */

/** 
 * Common setup method for HoTTv4
 */
void setupHoTTV4() {
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

/* ##################################################################### *
 *                HoTTv4 Electrical Air Module                           *
 * ##################################################################### */

/**
 * Writes cell 1-3 high, low values and if not available 
 * calculates vbat.
 */
static void hottV4EAMUpdateBattery() {
  HoTTV4ElectricAirModule.cell1L = MultiHoTTModule.cell1;
  HoTTV4ElectricAirModule.cell1H = MultiHoTTModule.cell1;
  HoTTV4ElectricAirModule.cell2L = MultiHoTTModule.cell2;
  HoTTV4ElectricAirModule.cell2H = MultiHoTTModule.cell2;
  HoTTV4ElectricAirModule.cell3L = MultiHoTTModule.cell3;
  HoTTV4ElectricAirModule.cell3H = MultiHoTTModule.cell3;
  
  if (MultiHoTTModule.driveVoltage > 0) {
    HoTTV4ElectricAirModule.driveVoltage = MultiHoTTModule.driveVoltage;
  } else {
    // Divide by 5 to convert to HoTT interval
    uint16_t vbat = (MultiHoTTModule.cell1 + MultiHoTTModule.cell2 + MultiHoTTModule.cell3) / 5;
    HoTTV4ElectricAirModule.driveVoltage = vbat; 
  }

  if (MultiHoTTModule.driveVoltage <= MultiHoTTModuleSettings.alarmDriveVoltage) {
    HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4ElectricAirModule.alarmInverse |= 0x80; // Invert Voltage display
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
  HoTTV4ElectricAirModule.alarmInverse = 0x0;
  
  hottV4EAMUpdateBattery();
  
  HoTTV4ElectricAirModule.temp1 = 20;
  HoTTV4ElectricAirModule.temp2 = 20;
  HoTTV4ElectricAirModule.current = 0; 
  HoTTV4ElectricAirModule.altitude = OFFSET_ALTITUDE + MultiHoTTModule.altitude;
  HoTTV4ElectricAirModule.m2s = OFFSET_M2S; 
  HoTTV4ElectricAirModule.m3s = OFFSET_M3S;

  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/* ##################################################################### *
 *                HoTTv4 VARIO Module                                    *
 * ##################################################################### */

/**
 * Sends HoTTv4 capable VARIO telemetry frame.
 */
static void hottV4SendVARIO() {
  static uint32_t previousMillis = millis();

  static int16_t maxAltitude = OFFSET_ALTITUDE;
  static int16_t minAltitude = OFFSET_ALTITUDE;
  
  static int32_t previousRawAltitude = 0;
  static int32_t referenceRawAltitude = 0;

  if ((0 == referenceRawAltitude) && (0 != MultiHoTTModule.altitude)) {
    referenceRawAltitude = MultiHoTTModule.altitude;
  } else {
    /** Minimum data set for EAM */
    HoTTV4VarioModule.startByte = 0x7C;
    HoTTV4VarioModule.sensorID = HOTTV4_VARIO_SENSOR_ID;
    HoTTV4VarioModule.sensorTextID = HOTTV4_VARIO_SENSOR_TEXT_ID;
    HoTTV4VarioModule.endByte = 0x7D;
    /** ### */
    
    HoTTV4VarioModule.altitude = OFFSET_ALTITUDE + (int)((MultiHoTTModule.altitude - referenceRawAltitude) / 100);
    
    maxAltitude = max(maxAltitude, HoTTV4VarioModule.altitude);
    HoTTV4VarioModule.maxAltitude = maxAltitude;
  
    minAltitude = min(minAltitude, HoTTV4VarioModule.altitude);
    HoTTV4VarioModule.minAltitude = minAltitude;
  
    HoTTV4VarioModule.m1sResolution = 117 + (MultiHoTTModule.altitude - previousRawAltitude);
    HoTTV4VarioModule.m3sResolution = 117;
    HoTTV4VarioModule.m10sResolution = 117;
  
    previousRawAltitude = MultiHoTTModule.altitude;
  
    // Clear output buffer
    memset(&outBuffer, 0, sizeof(outBuffer));
    
    // Copy EAM data to output buffer
    memcpy(&outBuffer, &HoTTV4VarioModule, kHoTTv4BinaryPacketSize);
    
    // Send data from output buffer
    hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
  }
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
  hottV4WriteLine(0, " MULTIHoTT SETTINGS");

  char text[21];
  snprintf(text, 21, "ALARM VOLT : %2i.%1iV", MultiHoTTModuleSettings.alarmDriveVoltage / 10, MultiHoTTModuleSettings.alarmDriveVoltage % 10);
  hottV4WriteLine(2, text, row, col);
  
  snprintf(text, 21, "ALARM TEMP1:  %3iC", MultiHoTTModuleSettings.alarmTemp1);
  hottV4WriteLine(3, text, row, col);
 
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTv4ElectricalAirTextModule, kHoTTv4TextPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4TextPacketSize);
}

/**
 * Expects an array of at least |size| bytes. All bytes till |size| will be transmitted
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
            hottV4SendEAM();
            hottV4_state = IDLE;
            break;
          
          case HOTTV4_VARIO_SENSOR_ID:
            hottV4SendVARIO();
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
                MultiHoTTModuleSettings.alarmDriveVoltage -= 1;
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
                MultiHoTTModuleSettings.alarmDriveVoltage += 1;
              } else if (row == 3) {
                MultiHoTTModuleSettings.alarmTemp1 += 1;
              }
            } else {
              row = row < 3 ? row + 1 : row;
            }
            break;
          case HOTTV4_BUTTON_SET:
            col = col == 1 ? 0 : 1;
            writeSettings();
            break;
        }
        
        hottV4SendEAMText(row, col);
        hottV4_state = IDLE;
      }
    }
  }
}
