/******************************************
 * Monkey Altimeter v0.8
 * K.C. Budd -/- phreakmonkey@gmail.com
 ******************************************/

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include "Config.h"

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

#define BMP085_ADDRESS 0x77   // I2C address of BMP085
const unsigned char OSS = 2;  // BMP085 Oversampling Setting

// BMP085 Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 
short temperature;
long pressure;
int32_t altitude;


//  User setup defaults
//  These default values are only used if EEPROM is uninitalized.
#define EEPROM_VERSION 0x03
int16_t variance = VARIANCE;
int16_t arm_threshold = ARM_THRESHOLD;
uint16_t altres_options[11] = {1, 5, 10, 20, 25, 50, 100, 200, 250, 500, 1000};
uint8_t altres = 3;
int16_t alt_offset = 0;  // In feet
int16_t tmp_offset = 0;  // In tenths of a degree C.

#ifndef HEADLESS
// Rotary Encoder
static uint8_t enc_prev_pos = 0;
static uint8_t enc_flags    = 0;
int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
uint8_t enc_cur_pos = 0;
#endif

// Button
int8_t buttonState = HIGH;      // the debounced reading from the input pin
int8_t lastButtonState = HIGH;  // the current state from the input pin
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // the debounce time
unsigned long buttonPressTime = 0;  // Return value (1=short, 2=long)
boolean buttonLockOut = false;

// Screen definitions:
#define KOLLSMAN 0
#define ALTHOLD  1
#define TEMP     2
#define VSI      3
#define SETUP    4

// User Setup screens:
#define S_VARIANCE  5
#define S_ARM       6
#define S_RESOLUT   7
#define S_AOFFSET   8
#define S_TOFFSET   9
#define S_EXIT     10

uint8_t screen = KOLLSMAN;
boolean setupmode = false;

// State variables:
int32_t althold = 0;
float slp = 29.92; // Updated with Kollsman Window function
boolean armed = false;
boolean alarm = false;

unsigned long clockTimer = 0;
unsigned long blinkTimer = 0;
unsigned long VSItimer = 0;
int32_t VSIprevalt = 0;
int fpm;
int8_t button;


void setup()
{
  Wire.begin();
  delay(500);  // Give the LCD a bit to intialize...
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  delay(100);

  // BANNER
  lcd.clear();
  lcd.setCursor(0,0);  lcd.print("MonkeyAlt v1.0");
  lcd.setCursor(0,1);  lcd.print("phreakmonkey.com");

#ifndef HEADLESS
  readConfig();  // Load user defined values
#endif
  
  bmp085Calibration();
  // Get initial values
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  altitude = pressureToAlt(pressure);
  VSIprevalt = altitude;
  fpm = 0;

  // Set up input devices
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
#ifndef HEADLESS
  pinMode(ROTARY_A, INPUT);
  pinMode(ROTARY_B, INPUT);
  digitalWrite(ROTARY_A, HIGH);
  digitalWrite(ROTARY_B, HIGH);

  // get an initial reading on the encoder pins
  if (digitalRead(ROTARY_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(ROTARY_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }
#endif

  // LED
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, LOW);  
  digitalWrite(GREEN, LOW);

  delay(3500);
  lcd.clear();
}


// **** MAIN LOOP ****
void loop()
{
  if (millis() - clockTimer > 250) {  // ~4 times a second
    temperature = bmp085GetTemperature(bmp085ReadUT());
    pressure = bmp085GetPressure(bmp085ReadUP());
    altitude = ((altitude << 1) + altitude + pressureToAlt(pressure)) >> 2; // four element moving average for smoothness
    if (setupmode) setupOutput();
    else lcdOutput();
    alarmCheck();
    updateLED();
    clockTimer = millis();
  }
  
#ifndef HEADLESS
  if (millis() - VSItimer > 4000) computeVSI(); // every 4 seconds
  rotaryCheck();  // monitor for rotation
#endif

  buttonCheck();  // results in button: 1 = short press, 2 = long press

  if (alarm) UIalarm();
#ifndef HEADLESS
  else if (setupmode) UIsetup();
#endif
  else UInormal();
      
  if (blinkTimer && (millis() - blinkTimer > 100)) {
    blinkTimer = 0;
    lcd.backlight();
    digitalWrite(RED, LOW);
  }
}


// ***** UI Routines *****

// ***** INPUT MENU LOGIC *****
void UIalarm()
{
    switch(button) {
      case 1:  // disarm
        armed = false;
        alarm = false;
        break;
      case 2:  // disarm and disable
        armed = false;
        alarm = false;
        althold = 0;
        screen = ALTHOLD;
        break;
    }
    // Proceed in case we're adjusting something when the alarm goes off:
    button = 0;  UInormal();  
    return;
}


void UInormal()
{
  switch(button) {
  #ifndef HEADLESS
    case 1:  // Next Screen
      screen = (screen + 1) % 5;
      lcd.clear();
      blinklcd();
      return;
  #endif
    case 2:  // ALT HOLD shortcut or ENT SETUP
      lcd.clear();
      blinklcd();
  #ifndef HEADLESS
      if (screen == SETUP) {  // Enter SETUP mode
        lcd.clear();
        setupmode = true;
        althold = 0;
        armed = false;
        screen = S_VARIANCE;
        return;
      }
      althold = (altitude + altres_options[altres]/2) / altres_options[altres] * altres_options[altres]; // Round to altres
      screen = ALTHOLD;
  #else
      // In headless mode we simply toggle althold mode.
      if (althold) althold = 0;
      else althold = altitude;
  #endif
      armed = true;
      return;
  }
  #ifndef HEADLESS
  if (enc_action) {
    switch(screen) {
      case KOLLSMAN:
        if (enc_action > 0) slp += .01;
        else slp -= .01;
        slp = constrain(slp, 28.90, 31.10);
        altitude = pressureToAlt(pressure);  // Shortcut averaging during reference changes
        break;
      case ALTHOLD:
        if (enc_action > 0) althold += altres_options[altres];
        else althold -= altres_options[altres];
        althold = constrain(althold, 0, 35000);
        armed = 0;
        break;
        
    }
  }
  #endif
}

#ifndef HEADLESS
void UIsetup()
{
  switch(button) {
    case 1:  // Next Screen
      screen = ((screen - 4) % 6) + 5;  // rotate thru 5 -> 10
      lcd.clear();
      blinklcd();
      return;
    case 2:  // Save & exit shortcut
      writeConfig();
      setupmode = false;
      screen = KOLLSMAN;
      lcd.clear();
      lcd.blink();
      return;
  }
  if (enc_action) {
    switch(screen) {
      case S_VARIANCE:
        if (enc_action > 0) variance += 10;
        else variance -= 10;
        variance = constrain(variance, 10, 1000);
        break;
      case S_ARM:
        if (enc_action > 0) arm_threshold += 10;
        else arm_threshold -= 10;
        arm_threshold = constrain(arm_threshold, 10, 1000);
        break;
      case S_RESOLUT:
        if (enc_action > 0) altres += 1;
        else altres -= 1;
        altres = constrain(altres, 0, 10);
        break;
      case S_AOFFSET:
        if (enc_action > 0) alt_offset += 1;
        else alt_offset -= 1;
        alt_offset = constrain(alt_offset, -250, 250);
        break;
      case S_TOFFSET:
        if (enc_action > 0) tmp_offset += 1;
        else tmp_offset -= 1;
        tmp_offset = constrain(tmp_offset, -100, 100);
        break;
      case S_EXIT:
        if (enc_action > 0) {
          writeConfig();
          setupmode = false;
          screen = KOLLSMAN;
          lcd.clear();
          lcd.blink();
          return;
        }
        else {
          readConfig();  // Reload previous values
          setupmode = false;
          screen = KOLLSMAN;
          lcd.clear();
          lcd.blink();
          return;
        }
    }
  }
  return;
}
#endif


// ***** INPUT DEVICES SENSING *****
void buttonCheck()
{
  int8_t reading = digitalRead(BUTTON);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    lastButtonState = reading;
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {  // Legit change
      buttonState = reading;
      if (buttonState == LOW) buttonPressTime = millis();
      else {
        if (millis() - buttonPressTime < 2000) {
          button = 1;  // Short press return
          return;
        }
        buttonLockOut = false;
      }
    }
    else if (!buttonLockOut && buttonState == LOW && millis() - buttonPressTime >= 2000) {
      buttonLockOut = true;  // We've got it now.
      button = 2; // Long press return
      return;
    }
  }
  button = 0;
  return;
}

#ifndef HEADLESS
void rotaryCheck()
{
  enc_action = 0; // 1 or -1 if moved, sign is direction
  enc_cur_pos = 0;

  // Rotary encoder functions borrowed from code sample at https://learn.adafruit.com/trinket-usb-volume-knob/code
  
  // read in the encoder state first
  if (bit_is_clear(ROTARY_PORT, ROTARY_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(ROTARY_PORT, ROTARY_B)) {
    enc_cur_pos |= (1 << 1);
  }
 
  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      enc_flags = 0; // reset for next time
    }
  }
 
  enc_prev_pos = enc_cur_pos;
  return;
}
#endif


// ***** LCD OUTPUT ROUTINES *****
void setupOutput()
{
  // User SETUP Menu screens:
  char altDisplay[14];
  lcd.setCursor(0,0);
    
  switch(screen) {
    case S_VARIANCE:
      lcd.print("Set Alert at    ");
      lcd.setCursor(0,1);
      lcd.print("+/- ");
      lcd.print(variance);
      lcd.print(" ft       ");
      break;
    case S_ARM:
      lcd.print("Set Arm within  ");
      lcd.setCursor(0,1);
      lcd.print("+/- ");
      lcd.print(arm_threshold);
      lcd.print(" ft       ");
      break;
    case S_RESOLUT:
      lcd.print("Hold Resolution:");
      lcd.setCursor(0,1);
      lcd.print(altres_options[altres]);
      lcd.print(" ft           ");
      break;
    case S_AOFFSET:
      lcd.print("Altitude offset:");
      lcd.setCursor(0,1);
      if (alt_offset > 0) lcd.print("+");
      lcd.print(alt_offset);
      lcd.print(" ft           ");
      break;
    case S_TOFFSET:
      lcd.print("Temp offset:    ");
      lcd.setCursor(0,1);
      if (alt_offset > 0) lcd.print("+");
      lcd.print((tmp_offset / 10.0),1);
      lcd.print(" C.           ");
      break;
    case S_EXIT:
      lcd.print("Right to save,  ");
      lcd.setCursor(0,1);
      lcd.print("Left to cancel. ");                
      break;
  }
}


void lcdOutput()
{
  // LCD screens:
  char altDisplay[14];
  lcd.setCursor(0,0);
  lcd.print("Alt: ");
  if (altitude > 0) lcd.print(valToStr(altitude, altDisplay, 14, ','));
  else lcd.print(altitude);
  lcd.print("          ");

  lcd.setCursor(0,1);
  switch(screen) {
    case KOLLSMAN:
      lcd.print("SLP: ");
      lcd.print(slp);
      lcd.print("      ");
      break;
      
    case ALTHOLD:
      lcd.print("Hold: ");
      if (althold == 0) lcd.print("off");
      else lcd.print(althold);
      lcd.print("         ");
      break;
      
    case TEMP:
      lcd.print("T: ");
      lcd.print((temperature + tmp_offset) / 10.0, 1);
      lcd.print("C ");
      lcd.print(((temperature + tmp_offset) / 10.0 * 1.8 + 32.0), 1);
      lcd.print("F   ");
      break;
      
    case VSI:
      lcd.print("VSI: ");
      lcd.print(fpm);
      lcd.print("          ");
      break;
      
    case SETUP:
      lcd.print("Long press SETUP");
      break;
  }
  return;
}


void blinklcd()
{
  lcd.noBacklight();
  digitalWrite(RED, HIGH);
  blinkTimer = millis();
}


void updateLED()
{
  if (!althold) {
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, LOW);
    return;
  } 
  if (alarm) {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);
    return;
  }
  if (armed) {
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, HIGH);
    return;
  }
  analogWrite(RED, 75);
  digitalWrite(GREEN, HIGH);
  return;
}


void alarmCheck()
{
  alarm = false;
  if (althold) {
    if (armed) {
      if (abs(altitude - althold) > variance) { // ** ALARM  **
        alarm = true;
        blinklcd();
      }
    }
    else if (abs(altitude - althold) < arm_threshold) armed = true;
  }
}


// ***** EEPROM *****
void readConfig() 
{
  /*** EEPROM Values:
   *  int8_t EEPROM_VERSION  0x02
   *  int16_t variance       default: 50 
   *  int16_t arm_threshold  default: 50
   *  uint16_t altres        default: 20
   *  int16_t alt_offset     default: 0
   *  int16_t tmp_offset     default: 0
   ***/
  int8_t eeprom_ver; 
  uint16_t eeAddress = 0;
  EEPROM.get(eeAddress, eeprom_ver); eeAddress += sizeof(eeprom_ver);
  if (eeprom_ver != EEPROM_VERSION)
  {
    writeConfig();
    return;
  }
  EEPROM.get(eeAddress, variance); eeAddress += sizeof(variance);
  EEPROM.get(eeAddress, arm_threshold); eeAddress += sizeof(arm_threshold);
  EEPROM.get(eeAddress, altres); eeAddress += sizeof(altres);
  EEPROM.get(eeAddress, alt_offset); eeAddress += sizeof(alt_offset);
  EEPROM.get(eeAddress, tmp_offset); eeAddress += sizeof(tmp_offset);
  return;  
}


void writeConfig()
{
  int8_t eeprom_ver = EEPROM_VERSION;
  uint16_t eeAddress = 0;

  EEPROM.put(eeAddress, eeprom_ver); eeAddress += sizeof(eeprom_ver);
  EEPROM.put(eeAddress, variance); eeAddress += sizeof(variance);
  EEPROM.put(eeAddress, arm_threshold); eeAddress += sizeof(arm_threshold);
  EEPROM.put(eeAddress, altres); eeAddress += sizeof(altres);
  EEPROM.put(eeAddress, alt_offset); eeAddress += sizeof(alt_offset);
  EEPROM.put(eeAddress, tmp_offset); eeAddress += sizeof(tmp_offset);
  return;
}


// ***** COMPUTATION / LOGIC ROUTINES *****


void computeVSI()
{
  fpm = (int)((altitude - VSIprevalt)) / (int)((millis() - VSItimer) / 1000) * 60;
  VSIprevalt = altitude;
  VSItimer = millis();
}


//----- BMP085 Calculation Routines.  See BMP085 datasheet for details. -----
long pressureToAlt(long pres)
{
  long alt;
  long SLPinPA;
  SLPinPA = slp * 3386.389;
  //alt = (float)44330 * (1 - pow((float)pressure/SLPinPA, 0.190295)) * 3.28084;
  alt = (float)145439.6372 * (1 - pow((float)pressure/SLPinPA, 0.190295));
  return (alt + alt_offset);
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);

  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

// ***** MISCELLANEOUS 

/*
 ** valToStr
 *
 * Convert a value to a string of decimal digits with optional
 * thousands separators and a null termination.
 *
 * 'val'     is the value to convert
 * 'buf'     points to a buffer in which to place the characters
 * 'bufSize' gives the size of the buffer (should be >= 14)
 * 'sepChar' is the thousands separator (if non-zero)
 *
 * The return value will be null if the buffer is too small.  Otherwise,
 * the return value will point to the most significant digit of the string.
 *
 */
char * valToStr(uint32_t val, char *buf, uint8_t bufSize, char sepChar)
{
    // validate the parameters, return null on error
    if ((bufSize < 2) || (buf == (char *)0))
        return((char *)0);

    // put a null at the end of the buffer, adjust the size
    buf += bufSize--;
    *(--buf) = '\0';

    // special case: value equal zero
    if (val == 0)
    {
        *(--buf) = '0';
    }
    else
    {
        uint8_t digCnt = 0;

        // general case: possibly multiple digits with thousands separators
        while ((val != 0) && bufSize)
        {
            // add a thousands separator every three digits
            if (sepChar && (digCnt >= 3) && (bufSize > 1))
            {
                *(--buf) = sepChar;
                bufSize--;
                digCnt = 0;
            }

            // add another digit to the buffer
            *(--buf) = (char)(val % 10) | '0';
            digCnt++;
            bufSize--;

            // prepare for producing the next digit
            val /= 10;
        }
    }

    // return a pointer to the completed string
    return(buf);
}
