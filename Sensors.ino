#include "MultiHoTTModule.h"

#define CELL1 A3
#define CELL2 A2
#define CELL3 A1
#define CELL4 A0
//#define TEMP1 A6
//#define CURRENT A7

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
  // cell1 1000 Ohm and  3830 Ohm for 1S voltage measuring 
  // cell2 1000 Ohm and  7870 Ohm for 2S voltage measuring 
  // cell3 1000 Ohm and 11800 Ohm for 3S voltage measuring 
  // cell4 1000 Ohm and 15800 Ohm for 4S voltage measuring 
  // 326 == 3,26V
  sensorsAvoidJitter(CELL1);
  uint32_t val = analogRead(CELL1);
  val = val * cali_cell1;
  MultiHoTTModule.cell1 = val/1000;
  sensorsAvoidJitter(CELL2);
  val = analogRead(CELL2);
  val = val * cali_cell2;
  MultiHoTTModule.cell2 = val/1000;
  sensorsAvoidJitter(CELL3);
  val = analogRead(CELL3);
  val = val * cali_cell3;
  MultiHoTTModule.cell3 = val/1000;
  sensorsAvoidJitter(CELL4);
  val = analogRead(CELL4);
  val = val * cali_cell4;
  MultiHoTTModule.cell4 = val/1000;
  
  // 126 == 12,6V
  MultiHoTTModule.vbat1 = (MultiHoTTModule.cell1 + MultiHoTTModule.cell2 + MultiHoTTModule.cell3 + MultiHoTTModule.cell4) /10 ;

  #ifdef DEBUG_CELL
      i2c_OLED_set_line(1);
      i2c_OLED_Print("Cell-1: ");
      print_Cell(MultiHoTTModule.cell1);
      i2c_OLED_set_line(2);
      i2c_OLED_Print("Cell-2: ");
      print_Cell(MultiHoTTModule.cell2);
      i2c_OLED_set_line(3);
      i2c_OLED_Print("Cell-3: ");
      print_Cell(MultiHoTTModule.cell3);
      i2c_OLED_set_line(4);
      i2c_OLED_Print("Cell-4: ");
      print_Cell(MultiHoTTModule.cell4);
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
