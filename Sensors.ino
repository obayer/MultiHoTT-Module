#include "MultiHoTTModule.h"

#define VBAT_S1 A0
#define VBAT_S2 A1
#define VBAT_S3 A2

#define VBAT_UPDATE_INTERVAL 1000

void sensorsReadVBAT() {
  static uint32_t lastUpdate = 0;
  
  if ((millis() - lastUpdate) > VBAT_UPDATE_INTERVAL) {
    lastUpdate = millis();

    // Convert from [0;1024] to [0;256] interval
    MultiHoTTModule.cell1 = analogRead(VBAT_S1) >> 2;
    MultiHoTTModule.cell2 = analogRead(VBAT_S2) >> 2;
    MultiHoTTModule.cell3 = analogRead(VBAT_S3) >> 2;
  }
}
