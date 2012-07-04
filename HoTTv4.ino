#include "HoTTv4.h"
#include "MultiHoTTModule.h"

#define HOTTV4_RXTX 9
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

static void hottV4SendEAM() {
  /** Minimum data set for EAM */
  HottV4ElectricAirModule.startByte = 0x7C;
  HottV4ElectricAirModule.sensorID = HOTTV4_ELECTRICAL_AIR_SENSOR_ID;
  HottV4ElectricAirModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
  HottV4ElectricAirModule.endByte = 0x7D;
  /** ### */

  HottV4ElectricAirModule.driveVoltage = MultiHoTTModule.vbat;
  
  HottV4ElectricAirModule.height = 500 + MultiHoTTModule.height;
  HottV4ElectricAirModule.temp1 = 20;
  HottV4ElectricAirModule.temp2 = 20;
  HottV4ElectricAirModule.m2s = 72;
  HottV4ElectricAirModule.m3s = 120;

  // Copy EAM data to output buffer
  memcpy(&outBuffer, &HottV4ElectricAirModule, kHoTTv4BinaryPacketSize);
  
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
