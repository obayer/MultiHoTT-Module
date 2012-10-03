#include "MultiHoTTModule.h"

#define VBAT A3

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
void readVBAT() {
  sensorsAvoidJitter(VBAT);

  // 51kOhm and 33kOhm for 3S voltage measuring
  // 126 == 12,6V
  MultiHoTTModule.driveVoltage = (126l * analogRead(VBAT)) >> 10;
}
