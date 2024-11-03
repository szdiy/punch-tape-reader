/*
    Manual Paper Tape Reader with USB Serial (CDC) Interface
   
    For Sparkfun/Arduino Pro Micro (ATmega 32U4), 5V/16MHz version.
    Optical reader with LED/phototransistor pairs for each data bit and feed track;
    analog input of phototransistor signals for automatic threshold adaptation.
    ADC clock is increased to 1 MHz for approx. 50 kHz conversion rate with Arduino library.

    Jürgen Müller 22.12.2020
    juergen@e-basteln.de
    www.e-basteln.de


    *** LEDs

    FEED  flashes once for every valid feed hole detected during operation and calibration
    
    CAL   steady ON:        calibration mode is active
          flashing 1 Hz:    waiting for user to start calibration (press CAL button), or 10 s timeout
          flashing 10 Hz:   error during calibration process
          ON in read mode:  invalid brightness levels have been read (no clear high or low)


    *** Jumpers
     
    Three different tape widths can be set via jumpers. (Setting them is recommended 
    to avoid "CAL" indications from invalid detection levels at the tape edges, 
    and mask invalid bits from the output bytes.)

    Bits    Tape placement    Jumper settings
    8 bit   76543.210         5BIT=open TST=open
    7 bit   -6543.210         5BIT=set  TST=set
    5 bit   --432.10-         5BIT=set  TST=open

    Setting only the TST jumper will activate test mode, where the raw ADC readings
    from all 9 channels are printed on the output in an endless loop.


    *** Calibration

    ADC thresholds for distinguishing low vs. high bits are set in software,
    and can be calibrated by pushing the CAL button. Status output during calibration
    is provided via test messages (USB) and via the CAL LED.

    * Press CAL once: View current calibration.
      Current calibration values are printed via USB, 
      CAL LED blinks at 1 Hz.
    
    * Press CAL again within 10 seconds: Enter calibration mode.
      Prompts user to start feeding tape, 
      CAL LED on.

    * Feed tape, beginning within 10 seconds:
      Approx. 30 cm of sample tape are required.
      Ideally all data bits should be exercised in the sample, 
      but if no significant contrast is detected for a data channel, 
      averaged thresholds from the other channels will be used.
      CAL LED stays on, FEED LED begins to flash when valid feed holes are seen.

    * Calibration automatically ends when enough sample tape has been processed,
      or when one of the 10 second timeouts mentioned above is reached.
      Results and final status are printed via USB; 
      errors are also signalled by rapid flashing of the CAL LED for 1 second.
      CAL LED is then switched off. 
*/

/*
  Keyboard test

  For the Arduino Leonardo, Micro or Due

  Reads a byte from the serial port, sends a keystroke back.
  The sent keystroke is one higher than what's received, e.g. if you send a,
  you get b, send A you get B, and so forth.

  The circuit:
  - none

  created 21 Oct 2011
  modified 27 Mar 2012
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/KeyboardSerial
*/

#include "Keyboard.h"

#include <EEPROM.h>

// ===========================
// select whether TX is used for serial output (9600 Baud) or as a timing signal for debugging
// ===========================

#define TX_SERIAL 1

#if TX_SERIAL

#define SERIAL_WRITE(c) Serial1.write(c)
#define SERIAL_INIT(c) Serial1.begin(c)
#define TX_ON
#define TX_OFF

#else

#define TX_ON digitalWrite (OUT_TX, ON)
#define TX_OFF digitalWrite (OUT_TX, OFF)
#define SERIAL_WRITE(c)
#define SERIAL_INIT(c)

#endif

// ===========================
// Pinout definition.
// Note that these designate Arduino I/O numbers, not physical pins of the DIP package!
// ===========================

// analog inputs - 8 bit data plus transport track
#define AN_D0  A3
#define AN_D1  A2
#define AN_D2  A1
#define AN_D3  A6
#define AN_D4  A7
#define AN_D5  A8
#define AN_D6  A9
#define AN_D7  A10
#define AN_FEED  A0

// digital I/O and LEDs
#define LED_FEED LED_BUILTIN_TX   // TX LED (green) signals feed pulse
#define LED_CAL LED_BUILTIN_RX    // RX LED (yellow) signals calibration mode
#define IN_CAL  2                 // button to start calibration mode
#define IN_5BIT 7                 // jumper selects 5-bit mode (output >> 1) or 6-bit mode (together with TST jumper)
#define IN_INV  5                 // jumper selects inverted input levels (low intensity == logic 1)
#define IN_TST  3                 // jumper enables raw data output for testing
#define OUT_TX  1                 // general signaling & debugging - also serial TX

// I/O levels -- all digital inputs and outputs are active low 
#define ON 0
#define OFF 1

// ===========================
// global constants and variables
// ===========================

// calibration values per channel
struct calibRec {
  byte min;       // lowest value read
  byte max;       // highest value read
  byte mod;       // modulation depth
  byte low;       // values below this threshold are valid '0'
  byte high;      // values above this threshold are valid '1'
  byte mid;       // midpoint between min and max defines "best guess" at 0 or 1
  boolean est;    // this entry is interpolated from other channels
};

struct calibRec calibData [8];    // calibration records for 8 data channels
struct calibRec calibFeed;        // calibration data for Feed track

const unsigned addrFeed = 0;     // EEPROM address of calibFeed
const unsigned addrData = sizeof (calibRec);  // EEPROM address of calibData

byte inv = 0xFF;                  // inversion mask for inputs, toggled by INV jumper

// plausibility tests for acceptable signals during calibration
const byte minlevel = 100;        // minimum "max" value for acceptable signal level
//const byte minlevel = 7; 
const unsigned contrast = 80;     // maximum "min" value as percentage of "max" value for acceptable modulation

const long timeout = 10000;       // 10 s timeout at various calibration steps


// ===========================
// initialize 
// ===========================

void setup() 
{
  // initialize control over the keyboard:
  Keyboard.begin();
  // Initialize digital I/O pins.
  pinMode (IN_CAL, INPUT_PULLUP);
  pinMode (IN_INV, INPUT_PULLUP);
  pinMode (IN_5BIT, INPUT_PULLUP);
  pinMode (IN_TST, INPUT_PULLUP);
  pinMode (OUT_TX, OUTPUT);
  pinMode (LED_FEED, OUTPUT);
  pinMode (LED_CAL, OUTPUT);

  digitalWrite (LED_FEED, OFF);  
  digitalWrite (LED_CAL, OFF);  
  TX_OFF;
  
  // Initialize USB serial comm.
  // Don't check for availability of the serial port; USB might be used for power supply only
  Serial.begin(9600);                 // baud rate is a dummy, not relevant for CDC

  // Initialize serial TX if enabled
  SERIAL_INIT(9600);

  // set ADC clock prescaler. 
  // 1 MHz gives stable results, 2 MHz gets noisy on some channels
  // ADCSRA = (ADCSRA&0xF80)|0x03;    // 1/8 (2 MHz ADC clock)
  ADCSRA = (ADCSRA&0xF80)|0x04;       // 1/16 (1 MHz ADC clock)

  // Read calibration data from EEPROM. 
  EEPROM.get (addrData, calibData);
  EEPROM.get (addrFeed, calibFeed);
}

// ===========================
// Calibration
// ===========================

// ---------------------------
// Print current calibration values for Feed track
// ---------------------------

void cal_print_feed ()
{
  Serial.println  ();
  Serial.println  ("\tFeed track calibration");
  Serial.println  ("\tBit\tMin\tMax\tMod\tLow\tHigh"); 
  Serial.print    ("\tFeed\t");
  Serial.print    (calibFeed.min);  Serial.print ("\t");
  Serial.print    (calibFeed.max);  Serial.print ("\t");
  Serial.print    (calibFeed.mod);  Serial.print ("%\t");
  Serial.print    (calibFeed.low);  Serial.print ("\t");
  Serial.print    (calibFeed.high); Serial.println ();
}

// ---------------------------
// Print current calibration values for Data tracks
// ---------------------------

void cal_print_data ()
{
  byte i;
  Serial.println ();    
  Serial.println ("\tData calibration");
  Serial.println ("\tBit\tMin\tMax\tMod\tLow\tHigh"); 
  for (i=0; i<8; i++) {
    Serial.print ("\t");
    Serial.print (i);                  Serial.print ("\t");
    Serial.print (calibData[i].min);   Serial.print ("\t");
    Serial.print (calibData[i].max);   Serial.print ("\t");
    Serial.print (calibData[i].mod);   Serial.print ("%\t");
    Serial.print (calibData[i].low);   Serial.print ("\t");
    Serial.print (calibData[i].high);  Serial.print ("\t");
    if (calibData[i].est) Serial.print ("interpolated"); 
    Serial.println ();
  }
}

// ---------------------------
// Wait for button release and new button press.
// Blink LED at 1 Hz while waiting.
// Returns true if successful (no timeout)
// ---------------------------

boolean cal_button ()
{
  long t;
  long endtime = millis() + timeout;
  
  while ((digitalRead (IN_CAL) == ON) && ((t = millis()) < endtime))  // wait for button release or timeout
    digitalWrite (LED_CAL, ((endtime-t)/500) % 2);                    // blink LED @ 1Hz
  delay (100);                                                        // debounce time
  while ((digitalRead (IN_CAL) == OFF) && ((t = millis()) < endtime)) // wait for button press or timeout
    digitalWrite (LED_CAL, ((endtime-t)/500) % 2);                    // blink LED @ 1 Hz
  return millis() < endtime;                                          // no button pressed, timed out
}

void cal_error (char* msg)
{
  Serial.println ();
  Serial.print ("--- ");
  Serial.println (msg);
  for (int i=0; i<10; i++) {
    digitalWrite (LED_CAL, ON); delay (50);
    digitalWrite (LED_CAL, OFF); delay (50);    
  }
}

// ---------------------------
// Feed sensor calibration.
// Waits until valid modlation amplitude is seen, then follows 20 feed hole level changes.
// Returns true if successful (no timeout)
// ---------------------------

boolean cal_read_feed ()
{
  byte b;
  unsigned i;
  long endtime = millis() + timeout;
  
  // wait until minimum modulation depth established, or 10s timeout 
  // abort after timeout in feed calibration 
  calibFeed.max = 0;
  calibFeed.min = 255;  
  while (((calibFeed.max < minlevel) || (contrast*calibFeed.max < 100*calibFeed.min)) && (millis() < endtime)) {
    b = (analogRead (AN_FEED)>>2)^inv;
    if (b > calibFeed.max) calibFeed.max = b;
    else if (b < calibFeed.min) calibFeed.min = b;
  }
  // initial thresholds
  calibFeed.low = (2*calibFeed.min + calibFeed.max)/3;
  calibFeed.high = (calibFeed.min + 2*calibFeed.max)/3;

  // wait for 20 maxima and minima, or new 10s timeout
  if (millis() < endtime) {
    endtime = millis() + timeout;
    for (i=0; i<20; i++) {
      
      // wait for feed hole to pass
      while (( (b = (analogRead (AN_FEED)>>2)^inv) > calibFeed.low) && (millis() < endtime)) {
        if (b > calibFeed.max) calibFeed.max = b;
        else if (b < calibFeed.min) calibFeed.min = b;      
      }
      digitalWrite (LED_FEED, OFF);
      
      // wait for new feed hole to appear
      while (( (b = (analogRead (AN_FEED)>>2)^inv) < calibFeed.high) && (millis() < endtime)) {
        if (b > calibFeed.max) calibFeed.max = b;
        else if (b < calibFeed.min) calibFeed.min = b;      
      }
      digitalWrite (LED_FEED, ON);
      
      // update thresholds
      calibFeed.low = (2*calibFeed.min + calibFeed.max)/3;
      calibFeed.high = (calibFeed.min + 2*calibFeed.max)/3;
    }

    // calculate secondary cal parameters and print feed calibration
    calibFeed.mid = (calibFeed.min + calibFeed.max)/2;
    calibFeed.mod = 100 - (100*calibFeed.min)/calibFeed.max;
  }
  return (millis() < endtime);  
}

// ---------------------------
// Data sensor calibration.
// Reads 50 data bytes (triggered by feed track), sets min and max values in calibData record.
// Returns true if successful (no timeout).
// ---------------------------

boolean cal_read_data ()
{
  byte b;
  byte ad[8];
  unsigned i, j;
  long endtime;

  for (i=0; i<8; i++) {
    calibData[i].max = 0;
    calibData[i].min = 255;  
  }  
  
  for (j=0; j<50; j++) {

    // wait for feed hole to go and come, with 10s timeout
    endtime = millis() + timeout;
    while ((((analogRead (AN_FEED) >> 2)^inv) > calibFeed.low) && (millis() < endtime));  // wait for prior feed hole to pass
    digitalWrite (LED_FEED, OFF);
    while ((((analogRead (AN_FEED) >> 2)^inv) < calibFeed.high) && (millis() < endtime)); // wait for new feed hole
    digitalWrite (LED_FEED, ON);
    if (millis() >= endtime) break;

    // read all 8 data bits (quickly)
    ad[0] = analogRead (AN_D0) >> 2;
    ad[1] = analogRead (AN_D1) >> 2;
    ad[2] = analogRead (AN_D2) >> 2;
    ad[3] = analogRead (AN_D3) >> 2;
    ad[4] = analogRead (AN_D4) >> 2;
    ad[5] = analogRead (AN_D5) >> 2;
    ad[6] = analogRead (AN_D6) >> 2;
    ad[7] = analogRead (AN_D7) >> 2;  

    // process data
    for (i=0; i<8; i++) {
      b = ad[i]^inv;
      if (b > calibData[i].max) calibData[i].max = b;
      else if (b < calibData[i].min) calibData[i].min = b;
    }
  }
  return millis() < endtime;
}

// ---------------------------
// Data sensor post-processing.
// Identifies channels with valid modulation, calculates interpolated min and max values for invalid channels.
// Calculates secondary modulation data (thresholds, modulation depth) for all channels.
// Returns true if >= 4 channels with valid modulation were found.
// ---------------------------

boolean cal_process_data ()
{
  unsigned sum_max, sum_min, n_val, i;

  // identify data bits which meet minimum modulation criterion
  sum_max = 0; sum_min = 0; n_val = 0;
  for (i=0; i<8; i++) {
    if ((calibData[i].max > minlevel) && (contrast*calibData[i].max > 100*calibData[i].min)) {   // valid signal and modulation depth
      calibData[i].est = false;
      sum_max += calibData[i].max;
      sum_min += calibData[i].min;
      n_val++;
    } else {
      calibData[i].est = true;      
    }
  }

  // calculate interpolated values for non-modulated bits, 
  // plus secondary calibration values
  if (n_val > 0) {
    sum_max = sum_max/n_val;
    sum_min = sum_min/n_val;
  } else {
    sum_max = 1;  // avoid div by 0 below
    sum_min = 0;
  }
  for (i=0; i<8; i++) {
    if (calibData[i].est) {
      calibData[i].max = sum_max;
      calibData[i].min = sum_min;
    }
    calibData[i].low = (2*calibData[i].min + calibData[i].max)/3;     // 1/3 above min
    calibData[i].high = (calibData[i].min + 2*calibData[i].max)/3;    // 1/3 below max
    calibData[i].mid = (calibData[i].min + calibData[i].max)/2;       // halfway between min and max
    calibData[i].mod = 100 - (100*calibData[i].min)/calibData[i].max;
  }
  return (n_val >= 4);  
}

// ---------------------------
// Calibration main routine.
// Control flow, prints status messages via USB serial interface.
// ---------------------------

void calibrate ()
{
  boolean valid;
    
  // print current calibration status
  Serial.println ();
  Serial.println ("Current calibration:");
  cal_print_feed ();
  cal_print_data ();

  // wait for another button press
  Serial.println ();
  Serial.println ("*** Insert tape and press CAL button again to start new calibration,");
  Serial.println ("    or wait for 10 second timeout.");

  if (!cal_button ()) {
    Serial.println ();
    Serial.println ("--- Timeout - using current calibration");
    digitalWrite (LED_CAL, OFF);
    return;    
  }
  
  // feed sensor calibration
  digitalWrite (LED_CAL, ON);
  Serial.println ();
  Serial.println ("*** Calibrating feed sensor. Please feed tape...");
  
  if (cal_read_feed ()) {
    cal_print_feed ();
  } else {
    cal_error ("Timeout waiting for feed holes - using prior calibration");
    EEPROM.get (addrFeed, calibFeed);   
    return;
  }
  
  // data sensor calibration: read 50 data bytes
  Serial.println ();
  Serial.println ("*** Calibrating data sensors. Please keep feeding...");

  if (cal_read_data ()) {
    valid = cal_process_data ();
    cal_print_data ();
  } else{
    cal_error ("Timeout waiting for data - using prior calibration");
    EEPROM.get (addrData, calibData);   
    EEPROM.get (addrFeed, calibFeed);   
    return;
  }

  // check validity, store new or recover old calibration
  Serial.println ();
  if (valid) {         // need at least three data bits with actual modulation
    Serial.println ("--- Calibration complete and stored");
    EEPROM.put (addrData, calibData);
    EEPROM.put (addrFeed, calibFeed);    
  } else {
    cal_error ("Less than 4 valid data channels - using prior calibration");
    EEPROM.get (addrData, calibData);    
    EEPROM.get (addrFeed, calibFeed);    
  }
  Serial.println ();
  digitalWrite (LED_CAL, OFF);
}

// ===========================
// main loop
// ===========================

// ---------------------------
// Test mode (selected by TST jumper):
// Display all ADC values
// ---------------------------

void main_testmode ()
{
  byte ad[9];
  byte i;

  // capture all 9 inputs
  TX_ON;
  ad[0] = analogRead (AN_D0) >> 2;
  ad[1] = analogRead (AN_D1) >> 2;
  ad[2] = analogRead (AN_D2) >> 2;
  ad[3] = analogRead (AN_D3) >> 2;
  ad[4] = analogRead (AN_D4) >> 2;
  ad[5] = analogRead (AN_D5) >> 2;
  ad[6] = analogRead (AN_D6) >> 2;
  ad[7] = analogRead (AN_D7) >> 2;
  ad[8] = analogRead (AN_FEED) >> 2;
  TX_OFF;

  // display all ADC levels, with inversion as selected by INV jumper
  for (i=0; i<9; i++) {
    Serial.print (ad[i]^inv); Serial.print ("\t");
  }
  Serial.println ();
}

void printBinary(byte num) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((num >> i) & 1); // 打印每一位
  }
  Serial.println(); // 换行
}

// ---------------------------
// Standard operation:
// Read byte when triggered by feed track, transmit via USB serial (and RS-232 TX if enabled).
// Poll CAL button and call calibration routine when required.
// ---------------------------

void main_runmode ()
{
  byte ad[8];
  byte a, b;
  short int i;
  byte valid;
  
  // look for next feed hole.
  TX_ON;                                                    // ready to look for transport hole
  while (((analogRead (AN_FEED)>>2)^inv) > calibFeed.low)   // wait for prior feed hole to pass
    if (digitalRead (IN_CAL) == ON) calibrate ();           // calibration button pressed?
  digitalWrite (LED_FEED, OFF);
  
  while (((analogRead (AN_FEED)>>2)^inv) < calibFeed.high)  // wait for new feed hole
    if (digitalRead (IN_CAL) == ON) calibrate ();           // calibration button pressed?
  digitalWrite (LED_FEED, ON);
  TX_OFF;                                                   // found new transport hole

  // read all 8 data bits (quickly)
  TX_ON;                            // begin conversion
  ad[0] = analogRead (AN_D0) >> 2;
  ad[1] = analogRead (AN_D1) >> 2;
  ad[2] = analogRead (AN_D2) >> 2;
  ad[3] = analogRead (AN_D3) >> 2;
  ad[4] = analogRead (AN_D4) >> 2;
  ad[5] = analogRead (AN_D5) >> 2;
  ad[6] = analogRead (AN_D6) >> 2;
  ad[7] = analogRead (AN_D7) >> 2;
  TX_OFF;                           // end conversion

  // set valid mask to ignore bits at the edges of the tape
  if (digitalRead (IN_5BIT) == OFF)
  {
    //Serial.write("8bit mode");
    valid = 0xFF;       // 5-bit jumper open: 8-bit mode
  } 
  else if (digitalRead (IN_TST) == OFF) valid = 0x3E;   // 5-bit jumper set, TST jumper open: 5-bit mode
  else valid <= 0x7F;                                   // 5-bit jumper and TST jumper set: 7-bit mode
  
  // check for valid data, convert, set quality LED
  digitalWrite (LED_CAL, OFF);
  for (i=7; i>=0; i--) {
    b = b<<1;
    a = ad[i]^inv;
    if (a > calibData[i].mid) b |= 0x01;    // '1' bit value
    if ((a > calibData[i].low) && (a < calibData[i].high) && (valid & 0x80)) digitalWrite (LED_CAL, ON);    // signal an invalid state
    valid = valid << 1;
  }

  // output the byte to Serial (USB), and to Serial1 (TX) if enabled
  if (digitalRead (IN_5BIT) == OFF) {         // 5-bit jumper open: 8-bit mode
    //Serial.write(b);
    //SERIAL_WRITE(b);      
    //printBinary(b); 
    if(b>=32 && b<=126)  //visible ASCII  
      Serial.write(b);   
      Keyboard.write(b); 
  } else if (digitalRead (IN_TST) == OFF){    // 5-bit jumper set, TST jumper open: 5-bit mode
    Serial.write((b>>1) & 0x1F);
    SERIAL_WRITE((b>>1) & 0x1F);   
  } else {                                    // 5-bit jumper and TST jumper set: 7-bit mode
    Serial.write(b & 0x7F);
    SERIAL_WRITE(b & 0x7F);       
  }
}

// ---------------------------
// Main loop:
// Poll INV and TST jumpers, call appropriate operating routine
// ---------------------------

void loop() 
{
  // Raw ADC input values are high for dark phototransistors;
  // Values will be inverted in normal operation (hole -> light -> high inverted value -> logical 1).
  // If INV jumper is set, don't invert.
  if (digitalRead (IN_INV) == ON) inv = 0x00; 
  else inv = 0xFF;

  if ((digitalRead (IN_TST) == ON) and (digitalRead (IN_5BIT) == OFF)) {
    // if TST jumper is set and 5BIT open, activate test mode: 
    // display all ADC values continuously via USB
    Serial.write("in test mode");
    main_testmode ();
  } else { 
    // otherwise, read one byte triggered by feed track and write to USB
    main_runmode ();
  }
}
