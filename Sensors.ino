#include "MultiHoTTModule.h"

#define CELL1 A3
#define CELL2 A2
#define CELL3 A1
#define CELL4 A0
//#define TEMP1 A4
//#define CURRENT A4

/**
 * Reads VBAT value on anlog input VBAT and outputs it
 * in 0.1V steps to MultiHoTTModule.vbat.
 */
void sensorsReadVBAT() {
  // cell1 470 Ohm and    0 Ohm for 1S voltage measuring 
  // cell2 470 Ohm and  470 Ohm for 2S voltage measuring 1/1
  // cell3 470 Ohm and  940 Ohm for 3S voltage measuring 1/2
  // cell4 470 Ohm and 1410 Ohm for 4S voltage measuring 1/3
  // 126 == 12,6V
  uint32_t val = analogRead(CELL1);
  val = val * cali_cell1;
  MultiHoTTModule.cell1 = val * 0.00488;
  val = analogRead(CELL2);
  val = val * cali_cell2;
  MultiHoTTModule.cell2 = val * 0.00488;
  val = analogRead(CELL3);
  val = val * cali_cell3;
  MultiHoTTModule.cell3 = val * 0.00488;
  val = analogRead(CELL4);
  val = val * cali_cell4;
  MultiHoTTModule.cell4 = val * 0.00488;
  MultiHoTTModule.vbat = MultiHoTTModule.cell1 + MultiHoTTModule.cell2 + MultiHoTTModule.cell3 + MultiHoTTModule.cell4;
  #ifdef DEBUG
//      Serial.print("Cell-1: ");
//      Serial.println(MultiHoTTModule.cell1,DEC);
//      Serial.print("Cell-2: ");
//      Serial.println(MultiHoTTModule.cell2, DEC);
//      Serial.print("Cell-3: ");
//      Serial.println(MultiHoTTModule.cell3, DEC);
//      Serial.print("Cell-4: ");
//      Serial.println(MultiHoTTModule.cell4, DEC);
//      Serial.print("VBat  : ");
//      Serial.println(MultiHoTTModule.vbat, DEC);
  #endif
}

/**
 * Reads temperature value(s) on analog input TEMP1 
 */
void sensorsReadTemperatures() {
  #ifdef TEMP1
    
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
  MultiHoTTModule.current = ((analogRead(CURRENT) * 488l) - 250000) / 28;   
 #endif
}
