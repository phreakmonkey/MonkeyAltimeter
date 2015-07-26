/******* MonkeyAltimeter Configuration **********/

//***  Pick which sensor you're using: 
// * (uncomment only one)
// TODO: Add more sensors
// The BOSCH BMP085 and BMP180 are interchangable
#define BMP085

// I2C address for LCD controller
// common addresses seem to be 0x27 and 0x3F
#define LCDADDR 0x3F

// Does this unit have an LCD & Rotary encoder?  If not, uncomment HEADLESS:
//#define HEADLESS

/*** Default (non-eeprom) values.  These are mostly important in headless mode.
 *   In interactive mode they can be customized on the SETUP
 *   screens.
 *   VARIANCE = +/- altitude alert sensitivity
 *   ARM_THRESHOLD = when do we consider the altitude "reached"?
 */
#define VARIANCE 50
#define ARM_THRESHOLD 40

// Rotary Encoder pins / bus
// (Note: Both pins must be on the same bus, or you'll have to edit the
// code).  Reverse A & B pins if the encoder turns the "wrong way"

#define ROTARY_A 3
#define ROTARY_B 2
#define ROTARY_PORT PIND
// Pro Micro config:
//#define ROTARY_A 14
//#define ROTARY_B 15
//#define ROTARY_PORT PIND


// LED output pins:
#define RED   10
#define GREEN 11
//#define RED   10
//#define GREEN 9

// Button input pin
#define BUTTON 6
//#define BUTTON 5

