#include "SoftwareSerial.h"
#include "MultiHoTTModule.h"

/**
 * MultiHoTT-Module is a stand alone Arduino based Application that acts as a bridge between
 * MutliWii FlightController and HoTTv4 capable devices to transmit telemetry information. 
 *
 * by Oliver Bayer, 07/2012
 */

//#define DEBUG
#define LED 13

void setup() {
  pinMode(LED, OUTPUT);

  // Used for debuging and to communicate with MultiWii
  Serial.begin(115200);
  
  setupHoTTV4();
  setupAltitude();
  
  readSettings();
  checkSettings();
}

static void blink() {
  static uint8_t blink = LOW;
  static uint8_t cnt = 0;

  if (3 == cnt) {
    cnt = 0;
  } else {
    digitalWrite(LED, blink);
    blink = !blink;
    cnt++;
  }
}

void loop() {
  static uint32_t last = 0;
  static uint8_t state = 0;

  uint32_t now = millis();

  if ((now - last) > 200) {
    last = now;
    
    /** Be alive blink */
    blink();

    switch (state) {
      case 0:
        /** Read VBAT */
        readVBAT();
        state++;
        break;
      
      case 1:
        /** Read current Altitude */
        readAltitude();
        state++;
        break;

      default:
        state = 0;
    }
  }

  /** Request new data from MultiWii */
  //multiWiiRequestData();

  /** Send telemetry data via HoTTv4 */
  hottV4SendTelemetry();
}
