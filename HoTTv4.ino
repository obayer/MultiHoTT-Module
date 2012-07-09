#include "HoTTv4.h"
#include "MultiHoTTModule.h"

#define HOTTV4_TX_DELAY 1000

static uint8_t outBuffer[173];

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
  DDRB &= ~(1 << 1);
  PORTB |= (1 << 1);
}

/**
 * Enabels TX and disables RX
 */
static inline void hottV4EnableTransmitterMode() {
  DDRB |= (1 << 1);
}

/**
 * Writes out given byte to HoTT serial interface.
 * If in debug mode, data is also written to UART serial interface. 
 */
static void hottV4SerialWrite(uint8_t c) {
  #ifdef DEBUG
    Serial.print(c, HEX);
  #endif

  hottV4Serial.write(c);
}

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
  
  if (MultiHoTTModule.vbat > 0) {
    HoTTV4ElectricAirModule.driveVoltage = MultiHoTTModule.vbat;
  } else {
    uint16_t vbat = (MultiHoTTModule.cell1 + MultiHoTTModule.cell2 + MultiHoTTModule.cell3) / 5;
    HoTTV4ElectricAirModule.driveVoltage = vbat; 
  }

  if (HoTTV4ElectricAirModule.driveVoltage < 102) {
    HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;  
    HoTTV4ElectricAirModule.alarmInverse = 0x80;
  } else {
    HoTTV4ElectricAirModule.alarmInverse = 0x0;
    HoTTV4ElectricAirModule.alarmTone = 0x0; 
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

  hottV4EAMUpdateBattery();

  HoTTV4ElectricAirModule.height = 500 + MultiHoTTModule.height;
  HoTTV4ElectricAirModule.temp1 = 20;
  HoTTV4ElectricAirModule.temp2 = 20;
  HoTTV4ElectricAirModule.m2s = 72;
  HoTTV4ElectricAirModule.m3s = 120;

  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HoTTV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
  // Send data from output buffer
  hottV4SendData(outBuffer, kHoTTv4BinaryPacketSize);
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
    TEXTMODE,
  } hottV4_state = IDLE;

  if (hottV4Serial.available() > 1) { 
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t c = hottV4Serial.read();
      
      if (IDLE == hottV4_state) {
       hottV4_state = (0x80 == c) ? BINARY : IDLE; 
      } else if (BINARY == hottV4_state) {
        switch (c) {
          case HOTTV4_ELECTRICAL_AIR_SENSOR_ID:
            hottV4SendEAM();
            hottV4_state = IDLE;
            break;

          default:
            hottV4_state = IDLE;
        }
      }
    }
  }
}
