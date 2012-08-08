#include "MultiHoTTModule.h"

#define VBAT A3
//#define TEMP1 A4
#define CURRENT A2

/** To avoid sensor jitter as found in: 
 * http://forums.adafruit.com/viewtopic.php?f=25&t=11597
 */
static inline void sensorsAvoidJitter(uint8_t pin) {
  analogRead(pin);
  delay(25);
}

/**
 * Reads VBAT value on anlog input VBAT and outputs it
 * in 0.1V steps to MultiHoTTModule.vbat.
 */
void sensorsReadVBAT() {
  sensorsAvoidJitter(VBAT);

  // 51kOhm and 33kOhm for 3S voltage measuring
  // 126 == 12,6V
  MultiHoTTModule.vbat = (127l * analogRead(VBAT)) >> 10;
}

/**
 * Reads temperature value(s) on analog input TEMP1 
 */
void sensorsReadTemperatures() {
  #ifdef TEMP1
    sensorsAvoidJitter(TEMP1);
    
    analogRead(TEMP1);
    delay(25);
    
    // Usage of temperature sensor LM35
    // 5000mv / 1024steps = 4,88mv/step --> 0,488C/step
    MultiHoTTModule.temp = ((analogRead(TEMP1) * 488) / 1000) + 0.5;
  #endif
}

/**
 * Reads current on on analog input CURRENT and outputs it in mA
 * to MultiHoTTModule.current.
 */
void sensorsReadCurrent() {
 #ifdef CURRENT
  sensorsAvoidJitter(CURRENT);
  MultiHoTTModule.current = ((analogRead(CURRENT) * 488l) - 250000) / 28;   
 #endif
}
