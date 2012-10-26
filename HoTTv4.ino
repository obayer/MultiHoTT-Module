#include "HoTTv4.h"
#include "MultiHoTTModule.h"

#define HOTTV4_RXTX 3 
#define HOTTV4_TX_DELAY 1000

#define HOTTV4_BUTTON_DEC 0x0B
#define HOTTV4_BUTTON_INC 0x0D
#define HOTTV4_BUTTON_SET 0x09
#define HOTTV4_BUTTON_NIL 0x0F
#define HOTTV4_BUTTON_NEXT 0x0E
#define HOTTV4_BUTTON_PREV 0x07

#define OFFSET_ALTITUDE 500
#define OFFSET_M2S 72
#define OFFSET_M3S 120

static uint8_t outBuffer[173];

static int32_t m1s = 0;
static int32_t m3s = 0;
static int32_t m10s = 0;

int16_t altitude = 0;

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

  if (MultiHoTTModule.driveVoltage <= MultiHoTTModuleSettings.minDriveVoltage) {
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
 * Seconds since start
 */
static uint32_t seconds() {
  return millis() / 1000;
}

/**
 * Updates m1s, m3s, and m10s inclination for VARIO.
 */
static void updateVarioInclination() {
  static uint32_t now_1 = 0;
  static uint32_t now_3 = 0;
  static uint32_t now_10 = 0;

  static int32_t reference_m1s = 0;
  static int32_t reference_m3s = 0;  
  static int32_t reference_m10s = 0;

  static int16_t m1s = 0;
  static int16_t m3s = 0;
  static int16_t m10s = 0;

  uint32_t now = seconds();

  if ((0 == reference_m1s) && (0 != MultiHoTTModule.altitude)) {
    reference_m1s = MultiHoTTModule.altitude;
    reference_m3s = MultiHoTTModule.altitude;    
    reference_m10s = MultiHoTTModule.altitude;
  } else {
    if ((now - now_1) >= 1) {
      m1s = (int16_t)(MultiHoTTModule.altitude - reference_m1s);
      reference_m1s = MultiHoTTModule.altitude;
      now_1 = now;      
    }

    if ((now - now_3) >= 3) {
      m3s = (int16_t)(MultiHoTTModule.altitude - reference_m3s);
      reference_m3s = MultiHoTTModule.altitude;
      now_3 = now;      
    }
    
    if ((now - now_10) >= 10) {
      m10s = (int16_t)(MultiHoTTModule.altitude - reference_m10s);
      reference_m10s = MultiHoTTModule.altitude;
      now_10 = now;      
    }
  }

#if defined DEBUG
  Serial.print("m1/s: ");
  Serial.println(m1s);
  Serial.print("m3/s: ");
  Serial.println(m3s);
  Serial.print("m10/s: ");
  Serial.println(m10s);
#endif
  
  if (MultiHoTTModuleSettings.varioBeep) {
    if (m1s >= 250) {
      HoTTV4VarioModule.alarmTone = 'D';
    } else if (m1s >= 100) {
      HoTTV4VarioModule.alarmTone = 'E';
    } else if (m1s <= -250) {
      HoTTV4VarioModule.alarmTone = 'N';
    } else if (m1s <= -100) {
      HoTTV4VarioModule.alarmTone = 'R';
    }
  }

  HoTTV4VarioModule.m1s = 30000 + m1s;
  HoTTV4VarioModule.m3s = 30000 + m3s;
  HoTTV4VarioModule.m10s = 30000 + m10s;
}

/**
 * Updates height over ground, max. height over ground, and min. height over ground for VARIO.
 */
static void updateVarioAltitude() {
  static int16_t maxAltitude = OFFSET_ALTITUDE;
  static int16_t minAltitude = OFFSET_ALTITUDE;
  
  static int32_t referenceRawAltitude = 0;

  if ((0 == referenceRawAltitude) && (0 != MultiHoTTModule.altitude)) {
    referenceRawAltitude = MultiHoTTModule.altitude;    
  } else {
    altitude = (int16_t)((MultiHoTTModule.altitude - referenceRawAltitude) / 100);
    maxAltitude = max(maxAltitude, OFFSET_ALTITUDE + altitude);
    minAltitude = min(minAltitude, OFFSET_ALTITUDE + altitude);
    
    if (altitude >= (100 * MultiHoTTModuleSettings.maxAltitude)) {
      HoTTV4VarioModule.alarmTone = HoTTv4NotificationMaxAltitude;  
      HoTTV4VarioModule.alarmInverse |= 0x2; // Invert max altitude 
    }
  }
  Serial.println(altitude);
  HoTTV4VarioModule.altitude = OFFSET_ALTITUDE + altitude; 
  HoTTV4VarioModule.maxAltitude = maxAltitude;
  HoTTV4VarioModule.minAltitude = minAltitude;  
}

/**
 * Sends HoTTv4 capable VARIO telemetry frame.
 */
static void hottV4SendVARIO() {
  /** Minimum data set for EAM */
  HoTTV4VarioModule.startByte = 0x7C;
  HoTTV4VarioModule.sensorID = HOTTV4_VARIO_SENSOR_ID;
  HoTTV4VarioModule.sensorTextID = HOTTV4_VARIO_SENSOR_TEXT_ID;
  HoTTV4VarioModule.endByte = 0x7D;
  /** ### */
  
  updateVarioAltitude();
  updateVarioInclination();

  // Clear output buffer
  memset(&outBuffer, 0, sizeof(outBuffer));
  
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4VarioModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
}

/* ##################################################################### *
 *                HoTTv4 Text Mode                                       *
 * ##################################################################### */

static void hottV4ClearAllTextLines() {
  memset(&HoTTv4TextModule.text[0], ' ', 8*21);
}

/**
 * Writes out a single text line of max. 21 chars into HoTTv4ElectricalAirTextModule
 */
static void hottV4WriteLine(uint8_t line, const char *text) {
  uint8_t writeText = 1;

  for (uint8_t index = 0; index < 21; index++) {
    if (0x0 != text[index] && writeText) {
       HoTTv4TextModule.text[(line * 21) + index] = text[index];
    } else {
      writeText = 0;
      HoTTv4TextModule.text[(line * 21) + index] = ' ';
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
 * Sends HoTTv4 capable VARIO text frame.
 */
static void hottV4SendVARIOText(uint8_t row, uint8_t col) {
  /** Minimum data set for EAM Text mode */
  HoTTv4TextModule.startByte = 0x7B;
  HoTTv4TextModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTv4TextModule.endByte = 0x7D;

  // Clear output buffer
  memset(&outBuffer, 0x0, sizeof(outBuffer));
  
  hottV4ClearAllTextLines();
  hottV4WriteLine(0, " MULTIHoTT VARIO");

  char text[21];

  if (MultiHoTTModuleSettings.varioBeep > 0) {
    snprintf(text, 21, "BEEPER   :     %s", "ON");
  } else {
    snprintf(text, 21, "BEEPER   :     %s", "OFF");
  }
  hottV4WriteLine(2, text, row, col);
 
  snprintf(text, 21, "MAX. ALT.:  %4im", MultiHoTTModuleSettings.maxAltitude);
  hottV4WriteLine(3, text, row, col);
 
  //snprintf(text, 21, "MAX. M1S :  %4im", MultiHoTTModuleSettings.maxAltitude);
  //hottV4WriteLine(3, text, row, col);
 
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTv4TextModule, kHoTTv4TextPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4TextPacketSize);
}

/**
 * Sends HoTTv4 capable EAM text frame.
 */
static void hottV4SendEAMText(uint8_t row, uint8_t col) {
  /** Minimum data set for EAM Text mode */
  HoTTv4TextModule.startByte = 0x7B;
  HoTTv4TextModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HoTTv4TextModule.endByte = 0x7D;

  // Clear output buffer
  memset(&outBuffer, 0x0, sizeof(outBuffer));
  
  hottV4ClearAllTextLines();
  hottV4WriteLine(0, " MULTIHoTT ELECTRIC");

  char text[21];
  snprintf(text, 21, "ALARM VOLT : %2i.%1iV", MultiHoTTModuleSettings.minDriveVoltage / 10, MultiHoTTModuleSettings.minDriveVoltage % 10);
  hottV4WriteLine(2, text, row, col);
  
  snprintf(text, 21, "MAX. TEMP:  %3iC", MultiHoTTModuleSettings.maxTemp1);
  hottV4WriteLine(3, text, row, col);
 
  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTv4TextModule, kHoTTv4TextPacketSize);
  
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
 * Evaluate input from sender in VARIO text mode.
 * @param c Given input data from sender.
 */
static void hottV4VARIOTextEvaluation(uint8_t c) {
  static uint8_t row = 2;
  static uint8_t col = 0;

  switch (c & 0x0F) {
    case HOTTV4_BUTTON_NEXT:
      break;
    case HOTTV4_BUTTON_PREV:
      break;
    case HOTTV4_BUTTON_DEC:
      if (col) {
        if (row == 2) {
          MultiHoTTModuleSettings.varioBeep = 0;
        } else if (row == 3) {
          MultiHoTTModuleSettings.maxAltitude -= 5;
        }
      } else {
        row = (row == 2) ? 3 : 2;
      }
      
      break;
    case HOTTV4_BUTTON_INC:
      if (col) {
        if (row == 2) {
          MultiHoTTModuleSettings.varioBeep = 1;
        } else if (row == 3) {
          MultiHoTTModuleSettings.maxAltitude += 5;
        }
      } else {
        row = (row == 2) ? 3 : 2;
      }

      break;
    case HOTTV4_BUTTON_SET:
      col = col == 1 ? 0 : 1;
      break;
  }

  hottV4SendVARIOText(row, col);
}

/**
 * Evaluate input from sender in EAM text mode.
 * @param c Given input data from sender.
 */
static void hottV4EAMTextEvaluation(uint8_t c) {
  static uint8_t row = 2;
  static uint8_t col = 0;

  switch (c & 0x0F) {
    case HOTTV4_BUTTON_NEXT:
      break;
    case HOTTV4_BUTTON_PREV:
      break;
    case HOTTV4_BUTTON_DEC:
      if (col) {
        if (row == 2) {
          MultiHoTTModuleSettings.minDriveVoltage -= 1;
        } else if (row == 3) {
          MultiHoTTModuleSettings.maxTemp1 -= 1;
        }
      } else {
        row = row > 2 ? row - 1 : row;
      }
      break;
    case HOTTV4_BUTTON_INC:
      if (col) {
        if (row == 2) {
          MultiHoTTModuleSettings.minDriveVoltage += 1;
        } else if (row == 3) {
          MultiHoTTModuleSettings.maxTemp1 += 1;
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
        if ((c & 0x0F) == HOTTV4_BUTTON_SET) {
          writeSettings();
        }
        
        switch (c & 0xF0) {
          case HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID:
            hottV4EAMTextEvaluation(c);
            break;

          case HOTTV4_VARIO_SENSOR_TEXT_ID:
            hottV4VARIOTextEvaluation(c);
            break;
        }

        hottV4_state = IDLE;
      }
    }
  }
}
