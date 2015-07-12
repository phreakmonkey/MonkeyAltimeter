/******* MonkeyAltimeter Configuration **********/

//***  Pick which sensor you're using:
// * (uncomment only one)
// TODO: Add more sensors (like BMP180)

#define BMP085

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
// code)
#define ROTARY_A 2
#define ROTARY_B 3
#define ROTARY_PORT PIND

// LED output pins:
#define RED   10
#define GREEN 11

// Button input pin
#define BUTTON 9

