#include "MultiHoTTModule.h"

#define VBAT A0
#define TEMP1 A4

/**
 * Reads VBAT value on anlog input VBAT.
 */
void sensorsReadVBAT() {
  analogRead(TEMP1);
  delay(25);

  // Convert from [0;1024] to [0;256] interval
  MultiHoTTModule.vbat = analogRead(VBAT) >> 2;
}

/**
 * Reads temperature value(s) on analog input TEMP1 
 */
void sensorsReadTemperatures() {
  #ifdef TEMP1
    // To avoid sensor jitter as found in http://forums.adafruit.com/viewtopic.php?f=25&t=11597
    analogRead(TEMP1);
    delay(25);
    
    // Usage of temperature sensor LM35
    // 5000mv / 1024steps = 4,88mv/step --> 0,488C/step
    // Actually the correct conversation rate would be 0,488 but 0,5 is close enough
    MultiHoTTModule.temp = ((analogRead(TEMP1) * 488) / 1000) + 0.5;
    Serial.println(MultiHoTTModule.temp, DEC);
  #endif
}
