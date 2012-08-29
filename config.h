/**
 * MultiHoTT-Module is a stand alone Arduino based Application that acts as a bridge between
 * MutliWii FlightController and HoTTv4 capable devices to transmit telemetry information. 
 *
 * by Oliver Bayer & Carsten Giesen, 07/2012
 */

//#define DEBUG_HOTT
#define DEBUG_MWii

/**
 * MultiWii has VBat function
 */
#define MultiWii_VBat

/**
 * MultiWii has GPS function
 */
//#define MultiWii_GPS

/**
 * Cell measurement calibration 
 * cell1 1000 Ohm and  3830 Ohm for 1S voltage measuring 
 * cell2 1000 Ohm and  7870 Ohm for 2S voltage measuring 
 * cell3 1000 Ohm and 11800 Ohm for 3S voltage measuring 
 * cell4 1000 Ohm and 15800 Ohm for 4S voltage measuring 
 * 326 == 3,26V
 */
#define DEBUG_CELL
#define cali_cell1  517 
#define cali_cell2  484 
#define cali_cell3  457 
#define cali_cell4  452


/**************************************************************************************/
/***********************          OLED - display settings         *********************/
/**************************************************************************************/


/*****************************   The type of LCD     **********************************/
/* choice of LCD attached for configuration and telemetry, see notes below */
#define OLED_I2C_128x64 // I2C LCD: OLED http://www.multiwii.com/forum/viewtopic.php?f=7&t=1350

/******************************   Logo settings     ***********************************/
//#define SUPPRESS_OLED_I2C_128x64LOGO  // suppress display of OLED logo to save memory

/**********************************    I2C speed   ************************************/
#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
//#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

/***************************    Internal i2c Pullups   ********************************/
/* enable internal I2C pull ups (in most cases it is better to use external pullups) */
//#define INTERNAL_I2C_PULLUPS


#define DEBUG

