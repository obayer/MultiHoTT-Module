#include "MultiHoTTModule.h"

/**
 * Requests via serial interface data from MultWii (>= 2.1) and stores requested
 * data in MultiHoTTModule struct. Requests at least REQUEST_DATA_DELAY milliseconds 
 * a new data frame. Frames are requested in Round Robin.
 */

#define REQUEST_DATA_DELAY 250 
#define INPUT_BUFFER_SIZE 64

#define MSP_IDENT     100
#define MSP_STATUS    101
#define MSP_BAT       110
#define MSP_ALTITUDE  109

static uint8_t inBuffer[INPUT_BUFFER_SIZE];

const static uint8_t schedule[] = { MSP_BAT, MSP_ALTITUDE };

/**
 * Main method of MultiWii integration.
 */
void multiWiiRequestData() {
  static uint32_t previousMillis = 0;
  static uint8_t index = 0; 
  
  if ((millis() - previousMillis) > REQUEST_DATA_DELAY) {
    previousMillis = millis();
    
    if (Serial.available() == 0) {
      mwRequestData(schedule[index++ % sizeof(schedule)]);
    } else {
      mwEvaluateResponse();
    }
  }    
}

/**
 * Sends a request to MultiWii with given cmd
 */
static void mwRequestData(uint8_t cmd) {
  char cmdBuffer[6] = { '$', 'M', '<', 0x0, cmd, cmd }; 

  if (0 == Serial.available()) {
    for (uint8_t i = 0; i < 6; i++) {
      Serial.write(cmdBuffer[i]);
    }
  } 
}

/**
 * Reads a MultWii command from serial interface and stores result in
 * inBuffer.
 */
void mwEvaluateResponse() {
  uint8_t c;
  uint8_t cmd;
  uint8_t checksum;
  uint8_t payloadSize;
  uint8_t offset;
  
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;
  
  while(Serial.available()) {
    c = Serial.read();

    if (IDLE == c_state) {
      c_state = ('$' == c) ? HEADER_START : IDLE;
    } else if (HEADER_START == c_state) {
      c_state = ('M' == c) ? HEADER_M : IDLE;
    } else if (HEADER_M == c_state) {
      c_state = ('>' == c) ? HEADER_ARROW : IDLE;
    } else if (HEADER_ARROW == c_state) {
      checksum = 0;
      offset = 0;
      memset(inBuffer, 0, sizeof(inBuffer));
      
      payloadSize = c;
      checksum ^= c;
      
      c_state = HEADER_SIZE;
    } else if (HEADER_SIZE == c_state) {
      cmd = c;
      checksum ^= c;
      
      c_state = HEADER_CMD;
    } else if (HEADER_CMD == c_state && offset < payloadSize) {
      checksum ^= c;
      inBuffer[offset++] = c; 
    } else if (HEADER_CMD == c_state && offset >= payloadSize) { 
      if (checksum == c) {
        mwEvaluateMSPResponse(cmd, inBuffer);
      }

      c_state = IDLE;
    }
  }
}

/**
 * Evaluates valid MultiWii Serial Protocol message and 
 * stores needed data for later transmission via HoTT. 
 */
static void mwEvaluateMSPResponse(uint8_t cmd, uint8_t *data) {
  switch(cmd) {
    case MSP_BAT:
      mwEvaluateMSP_BAT(data);
      break;
    case MSP_ALTITUDE:
      mwEvaluateMSP_ALTITUDE(data);
      break;
  }
}

/**
 * Reads VBAT from given MSP data frame and stores it for later usage.
 */
static void mwEvaluateMSP_BAT(uint8_t *data) {
  MultiHoTTModule.vbat = data[0];
}

/**
 * Reads altitude from MSP data frame and stores it for later usage.
 */
static void mwEvaluateMSP_ALTITUDE(uint8_t *data) {
}
