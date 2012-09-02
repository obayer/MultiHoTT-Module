#include "MultiHoTTModule.h"
#include "config.h"

/**
 * Requests via serial interface data from MultWii (>= 2.1) and stores requested
 * data in MultiHoTTModule struct. Requests at least REQUEST_DATA_DELAY milliseconds 
 * a new data frame. Frames are requested in Round Robin.
 */

#define REQUEST_DATA_DELAY 250 
#define INPUT_BUFFER_SIZE 128

#define MSP_IDENT     100   //out message         multitype + multiwii 
#define MSP_STATUS    101   //out message         cycletime & errors_count & sensor present & box activation 
#define MSP_BAT       110   //out message         vbat, powermetersum 
#define MSP_ALTITUDE  109   //out message         1 altitude 
#define MSP_RAW_GPS   106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS  107   //out message         distance home, direction home
#define MSP_ATTITUDE  108   //out message         2 angles 1 heading 
#define MSP_ALTITUDE  109   //out message         1 altitude 
#define MSP_HEADING   125   //out message         headings and MAG configuration 

static uint8_t inBuffer[INPUT_BUFFER_SIZE];
static uint8_t indRX;

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuffer[indRX++]&0xff;
}

const static uint8_t schedule[] = { MSP_BAT, MSP_ALTITUDE, 
                                    #ifdef MultiWii_GPS
                                      MSP_RAW_GPS, 
                                      MSP_COMP_GPS, 
                                    #endif
                                    MSP_HEADING };

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
  uint8_t c = 0;
  uint8_t cmd = 0;
  uint8_t checksum = 0;
  uint8_t payloadSize = 0;
  uint8_t offset = 0;
  
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
      indRX = 0;
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
      mwEvaluateMSP_BAT();
      break;
    case MSP_ATTITUDE:
      mwEvaluateMSP_ATTITUDE();
      break;
    case MSP_ALTITUDE:
      mwEvaluateMSP_ALTITUDE();
      break;
    #ifdef MultiWii_GPS
      case MSP_RAW_GPS:
        //mwEvaluateMSP_RAW_GPS();
        break;
      case MSP_COMP_GPS:
        mwEvaluateMSP_COMP_GPS();
        break;
    #endif
    case MSP_HEADING:
      mwEvaluateMSP_HEADING();
      break;
  }
}

/**
 * Reads VBAT from given MSP data frame and stores it for later usage.
 */
static void mwEvaluateMSP_BAT() {
  MultiHoTTModule.vbat2 = read8();  //data[0];
  MultiHoTTModule.intPowerMeterSum = read16();  //data[1]+(data[2]*0x100);
  #ifdef DEBUG_MWii
      LCD_set_line(3);
      LCD_Print("EAM ");
      print_VBAT(MultiHoTTModule.vbat1);
      LCD_Print(" ");
      print_VBAT(MultiHoTTModule.vbat2);
  #endif
}

/**
 * Reads altitude from MSP data frame and stores it for later usage.
 */
static void mwEvaluateMSP_ALTITUDE() {
  MultiHoTTModule.altitude = read32();  //data[0]+(data[1]*0x100)+(data[2]*0x10000)+(data[4]*0x1000000);
}

/**
 * Reads attitude from MSP data frame and stores it for later usage.
 */
static void mwEvaluateMSP_ATTITUDE() {
  MultiHoTTModule.attitudeAngles1 = read16();  //data[0]+(data[1]*0x100);
  MultiHoTTModule.attitudeAngles2 = read16();  //data[2]+(data[3]*0x100);
  MultiHoTTModule.attitudeHeading = read16();  //data[4]+(data[5]*0x100);
}

#ifdef MultiWii_GPS
  /**
   * Reads RAW_GPS from MSP data frame and stores it for later usage.
   //out message         fix, numsat, lat, lon, alt, speed
   */
//     headSerialReply(14);
//     serialize8(f.GPS_FIX);
//     serialize8(GPS_numSat);
//     serialize32(GPS_coord[LAT]);
//     serialize32(GPS_coord[LON]);
//     serialize16(GPS_altitude);
//     serialize16(GPS_speed); 
static void mwEvaluateMSP_RAW_GPS() {
    MultiHoTTModule.GPS_fix       = read8();   //data[0];
    MultiHoTTModule.GPS_numSat    = read8();   //data[1];
    MultiHoTTModule.GPS_latitude  = read32();  //data[2]+(data[3]*0x100)+(data[4]*0x10000)+(data[5]*0x1000000);
    MultiHoTTModule.GPS_longitude = read32();  //data[6]+(data[7]*0x100)+(data[8]*0x10000)+(data[9]*0x1000000);
    MultiHoTTModule.GPS_altitude  = read16();  //data[10]+(data[11]*0x100);
    MultiHoTTModule.GPS_speed     = read16();  //data[12]+(data[13]*0x100);
    #ifdef DEBUG_MWii
        LCD_set_line(4);
        print_GPSLine1(MultiHoTTModule.GPS_numSat);
        LCD_set_line(5);
        print_GPSLine2(MultiHoTTModule.GPS_longitude,MultiHoTTModule.GPS_latitude);
    #endif
  }
  
  /**
   * Reads COMP_GPS from MSP data frame and stores it for later usage.
   //out message         distance home, direction home
   */
  static void mwEvaluateMSP_COMP_GPS() {
  //     serialize16(GPS_distanceToHome);
  //     serialize16(GPS_directionToHome);
  //     serialize8(GPS_update & 1);   
    MultiHoTTModule.GPS_distanceToHome  = read16();  //data[0]+(data[1]*0x100);
    MultiHoTTModule.GPS_directionToHome = read16();  //data[2]+(data[3]*0x100);
    MultiHoTTModule.GPS_update          = read8();   //data[4];
    #ifdef DEBUG_MWii
        LCD_set_line(6);
        print_GPSLine3(MultiHoTTModule.GPS_directionToHome);
    #endif
  }
#endif

/**
 * Reads HEADING from MSP data frame and stores it for later usage.
 */
static void mwEvaluateMSP_HEADING() {
//     serialize8( f.MAG_MODE<<0 | f.HEADFREE_MODE<<1 );
//     serialize16(heading);
//     serialize16(magHold);
//     serialize16(headFreeModeHold);
  MultiHoTTModule.magMode          = read8();   //data[0];
  MultiHoTTModule.heading          = read16();  //data[1]+(data[2]*0x100);
  MultiHoTTModule.magHold          = read16();  //data[3]+(data[4]*0x100);
  MultiHoTTModule.headFreeModeHold = read16();  //data[5]+(data[6]*0x100);
}


