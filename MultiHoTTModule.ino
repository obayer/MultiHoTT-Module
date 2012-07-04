#include "SoftwareSerial.h"

/**
 * MultiHoTT-Module is a stand alone Arduino based Application that acts as a bridge between
 * MutliWii FlightController and HoTTv4 capable devices to transmit telemetry information. 
 *
 * by Oliver Bayer, 07/2012
 */

#define DEBUG

void setup() {
  Serial.begin(115200);
  
  hottV4Setup();
}

void loop() {
  /** Request new data from MultiWii */
  multiWiiRequestData();

  /** Send telemetry data via HoTTv4 */
  hottV4SendTelemetry();
}
