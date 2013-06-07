/*
 BulbdialClock.ino

Updates 2013 William B Phelps:
 - advance hour hand at minute>30
 - keep time as signed long instead of hr, min, sec
 - light 2 LED's while setting seconds, minutes to show odd numbers
 - lower low brightness
 - fade hour hand change
 - logarithmic fade option
 - stop time update for negative second adjustment
 - interrupt driven display
 - GPS support
Todo:
 - if GPS, set but ignore Chronodot?
 - auto dim/bright
 - time setting button repeat
 
 Software for the Bulbdial Clock kit designed by
 Evil Mad Scientist Laboratories: http://www.evilmadscientist.com/go/bulbdialkit
 
 Updated to work with Arduino 1.0 by Ray Ramirez
 Also requires Time library:  http://www.arduino.cc/playground/Code/Time
 
 Target: ATmega328, clock at 16 MHz (Arduino Duemilanove w/328)
 
 Version 1.0.1 - 1/14/2012
 Copyright (c) 2009 Windell H. Oskay.  All right reserved.
 http://www.evilmadscientist.com/
 
 This library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this library.  If not, see <http://www.gnu.org/licenses/>.
 
 */

#include <EEPROM.h>            // For saving settings
#include <Wire.h>              // For optional RTC module
#include <Time.h>              // For optional Serial Sync
#include "gps.h"

/*
 EEPROM variables that are saved:  7
 
 * Brightness setting (range: 1-8)  Default: 8   (Fully bright)
 
 * Red brightness  (range: 0-63)    Default: 20
 * Green brightness (range: 0-63)   Default: 63
 * Blue brightness (range: 0-63)    Default: 63
 
 * Time direction (Range: 0,1)      Default: 0  (Clockwise)
 * Fade style (Range: 0,3)         Default: 1  (Fade enabled)
 
 * Alignment mode                  Default: 0
 
 */

// "Factory" default configuration can be configured here:
#define MainBrightDefault 8
#define MainBrightOffset 31

#define RedBrightDefault 63  // Use 63, default, for kits with monochrome LEDs!
#define GreenBrightDefault 63
#define BlueBrightDefault 63

#define CCWDefault 0
#define FadeModeDefault 1
#define FadeModes 6

#define AlignModeDefault 0

#define TIME_MSG_LEN 11  // time sync to PC is HEADER followed by unix time_t as ten ascii digits
#define TIME_HEADER 255  // Header tag for serial time sync message

// The buttons are located at D5, D6, & D7.
#define buttonmask 224

// LED outputs B0-B2:
#define LEDsB 7

// C0-C3 are LED outputs:
#define LEDsC 15

// TX, PD2,PD3,PD4 are LED outputs.
#define LEDsD 28

// Negative masks of those LED positions, for quick turn-off:
#define LEDsBInv 248
#define LEDsCInv 240
#define LEDsDInv 227

#define LED_B_Off();   DDRB &= LEDsBInv;  PORTB &= LEDsBInv;
#define LED_C_Off();   DDRC &= LEDsCInv;  PORTC &= LEDsCInv;
#define LED_D_Off();   DDRD &= LEDsDInv;  PORTD &= LEDsDInv;

#define AllLEDsOff();  LED_B_Off(); LED_C_Off(); LED_D_Off();

#define fadeMax 63
#define fadeGamma 1.8
#define deltaGamma 0.6

// s-shaped log curve (Phelps/Johnson) - gamma 1.8, delta 0.6 - 04 June 2013 William B Phelps
//byte FadeConv[] = {0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10,
//byte FadeConv[] = {0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10,
// 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20,
// 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 24, 25, 25, 25, 26, 26, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30,
// 30, 31, 31, 31, 32, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 37, 38, 38, 38, 39, 39, 39, 40,
// 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 43, 43, 44, 44, 44, 45, 45, 45, 46, 46, 46, 47, 47, 47, 48, 48, 48, 49, 49, 49, 49,
// 50, 50, 50, 51, 51, 51, 52, 52, 52, 53, 53, 53, 54, 54, 54, 54, 55, 55, 55, 56, 56, 56, 57, 57, 57, 58, 58, 58, 59, 59, 59,
// 60, 60, 60, 60, 61, 61, 61, 62, 62, 62, 63, 63};

//byte FadeConv[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6,
//6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15,
//15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 18, 18, 18, 18, 19, 19, 19, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24,
//24, 24, 25, 25, 25, 26, 26, 26, 27, 27, 27, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35,
//35, 35, 36, 36, 37, 37, 37, 38, 38, 39, 39, 39, 40, 40, 40, 41, 41, 42, 42, 42, 43, 43, 44, 44, 45, 45, 45, 46, 46, 47, 47, 48,
//48, 48, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 54, 54, 55, 55, 56, 56, 57, 57, 58, 58, 59, 59, 59, 60, 60, 61, 61, 62, 62, 63};

byte FadeConv[] = {0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 23, 23, 23, 23, 24, 24, 24, 25, 25, 25, 25, 26, 26, 26, 27, 27, 27, 28, 28, 28, 28, 29, 29, 29, 30, 30, 30, 31, 31, 31, 32, 32, 32, 33, 33, 33, 34, 34, 34, 35, 35, 35, 35, 36, 36, 36, 37, 37, 38, 38, 38, 39, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 45, 45, 45, 46, 46, 46, 47, 47, 48, 48, 48, 49, 49, 49, 50, 50, 51, 51, 51, 52, 52, 53, 53, 53, 54, 54, 55, 55, 55, 56, 56, 57, 57, 57, 58, 58, 59, 59, 60, 60, 60, 61, 61, 62, 62, 63, 63};


void TakeHigh(byte LEDline)
{
  switch( LEDline )
  {
  case 1:
    DDRB  |= 4;
    PORTB |= 4;
    break;
  case 2:
    DDRC  |= 1;
    PORTC |= 1;
    break;
  case 3:
    DDRC  |= 2;
    PORTC |= 2;
    break;
  case 4:
    DDRC  |= 4;
    PORTC |= 4;
    break;
  case 5:
    DDRC  |= 8;
    PORTC |= 8;
    break;
  case 6:
    DDRD  |= 16;
    PORTD |= 16;
    break;
  case 7:
    DDRD  |= 4;
    PORTD |= 4;
    break;
  case 8:
    DDRB  |= 1;
    PORTB |= 1;
    break;
  case 9:
    DDRD  |= 8;
    PORTD |= 8;
    break;
  case 10:
    DDRB  |= 2;
    PORTB |= 2;
    break;
    // default:
  }
}


void TakeLow(byte LEDline)
{
  switch( LEDline )
  {
  case 1:
    DDRB  |= 4;
    PORTB &= 251;
    break;
  case 2:
    DDRC  |= 1;
    PORTC &= 254;
    break;
  case 3:
    DDRC  |= 2;
    PORTC &= 253;
    break;
  case 4:
    DDRC  |= 4;
    PORTC &= 251;
    break;
  case 5:
    DDRC  |= 8;
    PORTC &= 247;
    break;
  case 6:
    DDRD  |= 16;
    PORTD &= 239;
    break;
  case 7:
    DDRD  |= 4;
    PORTD &= 251;
    break;
  case 8:
    DDRB  |= 1;
    PORTB &= 254;
    break;
  case 9:
    DDRD  |= 8;
    PORTD &= 247;
    break;
  case 10:
    DDRB  |= 2;
    PORTB &= 253;
    break;
    // default:
  }
}


void printDigits(byte digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits,DEC);
}


void digitalClockDisplay(){
  // digital clock display of current date and time
  Serial.print(hour(),DEC);
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(dayStr(weekday()));
  Serial.print(" ");
  Serial.print(monthStr(month()));
  Serial.print(" ");
  Serial.println(day(),DEC);
}

const byte SecHi[30] = {
  2,3,4,5,6,1,3,4,5,6,1,2,4,5,6,1,2,3,5,6,1,2,3,4,6,1,2,3,4,5};
const byte SecLo[30] = {
  1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,4,4,5,5,5,5,5,6,6,6,6,6};

const byte MinHi[30] = {
  1,7,1,8,1,9,2,7,2,8,2,9,3,7,3,8,3,9,4,7,4,8,4,9,5,7,5,8,5,9};
const byte MinLo[30] = {
  7,1,8,1,9,1,7,2,8,2,9,2,7,3,8,3,9,3,7,4,8,4,9,4,7,5,8,5,9,5};

const byte HrHi[12]  = {
  10, 1, 2,10,10, 6, 3,10,10, 4, 5,10};
const byte HrLo[12]  = {
   1,10,10, 2, 6,10,10, 3, 4,10,10, 5};

//int8_t SecNow;
//int8_t MinNow;
//int8_t HrNow;
long timeNow;
byte HrDisp, MinDisp, SecDisp;
byte SecNext, MinNext, HrNext;

#define EELength 7
byte EEvalues[EELength];

// Variables to store brightness of the three LED rings.
byte HourBright;
byte MinBright;
byte SecBright;
byte MainBright;

unsigned long millisThen;
unsigned long millisNow;
byte TimeSinceButton;
byte LastSavedBrightness;
byte DisplayOn = true;

byte PINDLast;

// Modes:
byte CCW;
byte ExtRTC;
byte UpdateRTC;  // set to force RTC update
byte SleepMode;
byte FadeMode;

byte VCRmode;  // In VCR mode, the clock blinks at you because the time hasn't been set yet.
byte FactoryResetDisable;  // To make sure that we don't accidentally reset the settings...

byte SettingTime;
byte AlignMode;
byte OptionMode;
byte AlignValue;
char AlignRate;

byte AlignLoopCount;
byte StartingOption;

byte HoldTimeSet;
byte HoldOption;
byte HoldAlign;

byte MomentaryOverridePlus;
byte MomentaryOverrideMinus;
byte MomentaryOverrideZ;

unsigned long  prevtime;

byte HrFade1, HrFade2, MinFade1, MinFade2, SecFade1, SecFade2;
byte LH1, LH2, LM1, LM2, LS1, LS2;
volatile byte H0, H1, H2, H3, H4, H5;
volatile byte L0, L1, L2, L3, L4, L5;
volatile byte D0, D1, D2, D3, D4, D5;

void ApplyDefaults (void) {
  /*
   * Brightness setting (range: 1-8)  Default: 8  (Fully bright)
   * Red brightness  (range: 0-63)    Default: 20
   * Green brightness (range: 0-63)   Default: 63
   * Blue brightness (range: 0-63)    Default: 63
   * Time direction (Range: 0,1)      Default: 0  (Clockwise)
   * Fade style (Range: 0,1)          Default: 1  (Fade enabled)
   */

  MainBright = MainBrightDefault;
  HourBright = RedBrightDefault;
  MinBright = GreenBrightDefault;
  SecBright = BlueBrightDefault;
  CCW = CCWDefault;
  FadeMode = FadeModeDefault;

}


void delayTime(byte time)  // delay (time * 0.045) ms
{
  unsigned int delayvar;
  delayvar = 0;
  while (delayvar <=  time)
  {
    asm("nop");
    delayvar++;
  }
}


void adjustTime()
{
          Serial.print("adjustTime ");
          Serial.print(hour());
          Serial.print(":");
          Serial.print(minute());
          Serial.print(":");
          Serial.print(second());
          Serial.println("");
  // Set time to that given from PC or GPS
  timeNow = 3600L*hour() + 60*minute() + second();
  if (timeNow > 43200)
    timeNow -= 43200;  // 12 hour time in seconds

  // Print confirmation
  Serial.print("Clock synced at: ");
  Serial.println(timeNow,DEC);

  if(timeStatus() == timeSet) {  // update clocks if time has been synced

    if ( prevtime != now() )
    {
      if (ExtRTC)
        RTCsetTime(timeNow);

      timeStatus();  // refresh the Date and time properties
      digitalClockDisplay( );  // update digital clock
      prevtime = now();
    }
  }
}


boolean getPCtime() {
  // if time sync available from serial port, update time and return true
  while(Serial.available() >=  TIME_MSG_LEN ){  // time message consists of a header and ten ascii digits
    if( Serial.read() == TIME_HEADER ) {
      time_t pctime = 0;
      for(int i=0; i < TIME_MSG_LEN -1; i++){
        char c= Serial.read();
        if( c >= '0' && c <= '9'){
          pctime = (10 * pctime) + (c - '0') ;  // convert digits to a number
        }
      }
      setTime(pctime);  // Sync Arduino clock to the time received on the serial port
      return true;  // return true if time message received on the serial port
    }
  }
  return false;  // if no message return false
}


void EEReadSettings (void) {  // TODO: Detect ANY bad values, not just 255.

  byte detectBad = 0;
  byte value = 255;

  value = EEPROM.read(0);

  if (value > 8)
    detectBad = 1;
  else
    MainBright = value;  // MainBright has maximum possible value of 8.

  if (value == 0)
    MainBright = 1;  // Turn back on when power goes back on-- don't leave it dark.

  value = EEPROM.read(1);
  if (value > 63)
    detectBad = 1;
  else
    HourBright = value;

  value = EEPROM.read(2);
  if (value > 63)
    detectBad = 1;
  else
    MinBright = value;

  value = EEPROM.read(3);
  if (value > 63)
    detectBad = 1;
  else
    SecBright = value;

  value = EEPROM.read(4);
  if (value > 1)
    detectBad = 1;
  else
    CCW = value;

  value = EEPROM.read(5);
  if (value == 255)
    detectBad = 1;
  else
    FadeMode = value;

  if (detectBad)
    ApplyDefaults();

  LastSavedBrightness = MainBright;

}

// reduce EE writes by checking current value first
void EEUpdate(int loc, byte val)
{
  byte c = EEPROM.read(loc);
  if (c!=val)
    EEPROM.write(loc, val);
}

void EESaveSettings (void){
  //EEPROM.write(Addr, Value);

  // Careful if you use this function: EEPROM has a limited number of write
  // cycles in its life.  Good for human-operated buttons, bad for automation.

  EEUpdate(0, MainBright);
  EEUpdate(1, HourBright);
  EEUpdate(2, MinBright);
  EEUpdate(3, SecBright);
  EEUpdate(4, CCW);
  EEUpdate(5, FadeMode);

  LastSavedBrightness = MainBright;

  // Optional: Blink LEDs off to indicate when we're writing to the EEPROM
  Blink(100);  // Blink LEDs off to indicate saving data

}


void NormalTimeDisplay(void) {
byte SecNow, MinNow, HrNow;
unsigned int t;

  t = timeNow;
  HrNow = t/3600;
  t = t - 3600*HrNow;  // seconds in this hour
  MinNow = t/60;
  SecNow = t - 60*MinNow;

  SecDisp = (SecNow/2 + 15);  // Offset by 30 s to project *shadow* in the right place.
  if ( SecDisp > 29)
    SecDisp -= 30;
//  SecDisp >>= 1;  // Divide by two, since there are 30 LEDs, not 60.

  SecNext = SecDisp + 1;
  if (SecNext > 29)
    SecNext = 0;

  MinDisp = (MinNow/2 + 15);  // Offset by 30 m to project *shadow* in the right place.
  if ( MinDisp > 29)
    MinDisp -= 30;
//  MinDisp >>= 1;  // Divide by two, since there are 30 LEDs, not 60.

  MinNext = MinDisp + 1;
  if (MinNext > 29)
    MinNext = 0;

  HrDisp = (HrNow + 6);  // Offset by 6 h to project *shadow* in the right place.
  
  if ( (FadeMode == 2) || (FadeMode == 4)) {
    if ( (SettingTime == 0) && (MinNow > 30) )  // If half the hour has gone by, (wbp)
      HrDisp += 1;  // advance the hour hand (wbp)
    if ( (OptionMode == 5) && (SecNow & 1) )  // If setting Fade modes, show hour hand "wiggle"
      HrDisp += 1;
  }

  if ( HrDisp > 11)
    HrDisp -= 12;

  HrNext = HrDisp + 1;
  if (HrNext > 11)
    HrNext = 0;

}


void NormalFades(void) {
byte SecNow, MinNow, HrNow;
unsigned long msNow;
unsigned int t;

  t = timeNow;
  HrNow = t/3600;
  t = t - 3600*HrNow;  // seconds in this hour
  MinNow = t/60;
  SecNow = t - 60*MinNow;
  msNow = millisNow - millisThen;  // how far into this second are we, in ms?

  switch (FadeMode)
  {
  case 0:  // no fading
    break;
  case 1:  // straddle 2 LEDs for odd seconds
    if (SecNow & 1)  // ODD second
      SecFade2 = 32;
    break;
  case 2:  // straddle 2 LEDs, move hour hand at 31 minutes
    if (SecNow & 1)  // ODD second
      SecFade2 = 32;
    if (MinNow == 30)  // second half of the hour (wbp)
    {
      if (SecNow == 59)
        HrFade2 = (msNow*63/1000);  // fade hour hand to new position
    }
    break;
  case 3:  // original fading
  case 4:  // move hour hand at 31 minutes
    // Normal time display
    if (SecNow & 1)  // ODD second
      SecFade2 = (msNow*63/1000);
    if (MinNow & 1)  // ODD minute
    {
      if ((SecNow == 59) || SettingTime) {
        MinFade2 = SecFade2;
      }
    }
    // fade the hour hand to it's next position, starting either at Minute 59 or Minute 30
    if ( ((MinNow == 59) && (FadeMode == 3)) || ((MinNow == 30) && (FadeMode == 4)) )  // end of the hour or second half of the hour (wbp)
    {
      if (SecNow == 59){
        HrFade2 = SecFade2;
      }
    }
    break;
  case 5:  // continuous fading
    if (SecNow & 1)  // Odd second
      msNow += 1000;  
    SecFade2 = msNow*63/2000;
    if (MinNow & 1)  // Odd minute
        SecNow += 60;
    MinFade2 = SecNow*63/120;  // fade minute hand slowly
    HrFade2 = MinNow*63/60;  // fade hour hand slowly
    break;
  case 6:  // continuous logarithmic fading
    if (SecNow & 1)  // Odd second
      msNow += 1000;  
    SecFade2 = FadeConv[msNow/10];  // 0 to 63
    if (MinNow & 1)  // Odd minute
      SecNow += 60;
    MinFade2 = FadeConv[SecNow*5/3];  // fade minute hand slowly
    HrFade2 = FadeConv[MinNow*10/3];  // fade hour hand slowly
    break;
  }
  SecFade1 = 63 - SecFade2;      
  MinFade1 = 63 - MinFade2;
  HrFade1 = 63 - HrFade2; 
}

void AlignDisplay(void)
{
  if (AlignMode & 1) {  // ODD mode, auto-advances

    byte AlignRateAbs;  // Absolute value of AlignRate

    if (AlignRate >= 0){
      AlignRateAbs = AlignRate + 1;
    }
    else
      AlignRateAbs = -AlignRate;

    // Serial.println(AlignRateAbs,DEC);

    AlignLoopCount++;

    byte ScaleRate;
    if (AlignRateAbs > 2)
      ScaleRate = 10;
    else if (AlignRateAbs == 2)
      ScaleRate = 50;
    else
      ScaleRate = 250;

    if (AlignLoopCount > ScaleRate) {
      AlignLoopCount = 0;

      if (AlignRate >= 0)
        IncrAlignVal();
      else
        DecrAlignVal();

    }
  }

  SecDisp = (AlignValue + 15);  // Offset by 30 s to project *shadow* in the right place.
  if ( SecDisp > 29)
    SecDisp -= 30;

  MinDisp = SecDisp;
  HrDisp = (AlignValue + 6);  // Offset by 6 h to project *shadow* in the right place.

  if ( HrDisp > 11)
    HrDisp -= 12;
}


void OptionDisplay(void)
{
#define StartOptTimeLimit 30
  if (StartingOption < StartOptTimeLimit) {

    AlignLoopCount++;  // Borrowing a counter variable...

    if (AlignLoopCount > 3) {
      AlignLoopCount = 0;
      StartingOption++;

      if (OptionMode == 1)  // Red (upper) ring color balance
      {
        HrDisp++;
        if (HrDisp > 11)
          HrDisp = 0;
      }
      if (OptionMode == 2)  // Green (middle) ring color balance
      {
        MinDisp++;
        if (MinDisp > 29)
          MinDisp = 0;
      }
      if (OptionMode == 3)  // Blue (lower) ring color balance (wbp)
      {
        SecDisp++;
        if (SecDisp > 29)
          SecDisp = 0;
      }
      if (OptionMode > 3)  // CW vs CCW OR fade mode
      {
        StartingOption = StartOptTimeLimit;  // Exit this loop
      }
    }
  }  // end "if (StartingOption < StartOptTimeLimit){}"

  if (StartingOption >= StartOptTimeLimit) {

    if (OptionMode == 4)  // CW vs CCW 
    {
      MinDisp++;
      if (MinDisp > 29)
        MinDisp = 0;
      SecDisp = MinDisp;
    }
    else  // FadeMode adjust
      NormalTimeDisplay();

  }
}


void RTCsetTime(unsigned long timeIn)
{
byte secondIn, minuteIn, hourIn;

  hourIn = timeIn/3600;  // hours
  minuteIn = timeIn/60%60;  // minutes
  secondIn = timeIn%60;  // seconds
  
  Wire.beginTransmission(104);  // 104 is DS3231 device address
  Wire.write((byte)0);  // start at register 0

  byte ts = secondIn / 10;
  byte os = secondIn % 10;
  byte ss = (ts << 4) + os;

  Wire.write(ss);  // Send seconds as BCD

  byte tm = minuteIn / 10;
  byte om = minuteIn % 10;
  byte sm = (tm << 4 ) | om;

  Wire.write(sm);  // Send minutes as BCD

  byte th = hourIn / 10;
  byte oh = hourIn % 10;
  byte sh = (th << 4 ) | oh;

  Wire.write(sh);  // Send hours as BCD

  Wire.endTransmission();

}

byte RTCgetTime()
{ // Read out time from RTC module, if present
  // send request to receive data starting at register 0

  byte status = 0;
  Wire.beginTransmission(104);  // 104 is DS3231 device address
  Wire.write((byte)0);  // start at register 0
  Wire.endTransmission();
  Wire.requestFrom(104, 3);  // request three bytes (seconds, minutes, hours)

  int seconds, minutes, hours;
  unsigned int timeRTC;
  byte updatetime = 0;
  unsigned long tNow;

  while(Wire.available())
  {
    status = 1;
    seconds = Wire.read();  // get seconds
    minutes = Wire.read();  // get minutes
    hours = Wire.read();    // get hours
  }

  // IF time is off by MORE than two seconds, then correct the displayed time.
  // Otherwise, DO NOT update the time, it may be a sampling error rather than an
  // actual offset.
  // Skip checking if minutes == 0. -- the 12:00:00 rollover is distracting,
  // UNLESS this is the first time running after reset.

  // if (ExtRTC) is equivalent to saying,  "if this has run before"

  if (status){
    seconds = (((seconds & 0b11110000)>>4)*10 + (seconds & 0b00001111));  // convert BCD to decimal
    minutes = (((minutes & 0b11110000)>>4)*10 + (minutes & 0b00001111));  // convert BCD to decimal
    hours = (((hours & 0b00110000)>>4)*10 + (hours & 0b00001111));  // convert BCD to decimal (assume 24 hour mode)
    timeRTC = 3600L*hours + 60*minutes + seconds;  // Values read from RTC
    if (timeRTC > 43200)
      timeRTC -= 43200;  // 12 hour time in seconds

    // Optional: report time::
    // Serial.print(hours); Serial.print(":"); Serial.print(minutes); Serial.print(":"); Serial.println(seconds);

//    if ((minutes) && (MinNow) ) {
    if (minutes) {  // don't adjust if top of the hour
      if (abs(timeRTC - timeNow) > 2)  // only adjust if more than 2 seconds off
        updatetime = 1;
    }

    if (UpdateRTC)  // First time since power up
      updatetime = 1;

    if (updatetime)
    {
      timeNow = timeRTC;  // update time from RTC 
      UpdateRTC = 0;  // time has been set
    }
  }

  return status;
}


void setup()  // run once, when the sketch starts
{
  Serial.begin(9600);
  setTime(0);

  PORTB = 0;
  PORTC = 0;
  PORTD = 0;

  DDRB = 0;  // All inputs
  DDRC = 0;  // All inputs
  DDRD = _BV(1);  // All inputs except TX.

  PORTD = buttonmask;  // Pull-up resistors for buttons

//  SecNow = 0;
//  HrNow = 0;
//  MinNow = 0;
  timeNow = 0;
  millisNow = millis();
  SecDisp = 0;
  MinDisp = 0;

  EEReadSettings();

  /*
   MainBright = 8;  // 8 is maximum value.
   HourBright = 30;
   MinBright = 63;
   SecBright = 63;
   CCW = 0;  // presume clockwise, not counterclockwise.
   FadeMode = 1;  // Presume fading is enabled.
   */

  VCRmode = 1;  // Time is NOT yet set.
  FactoryResetDisable = 0;
  TimeSinceButton = 0;

  PINDLast =  PIND & buttonmask;
  // ButtonHold = 0;

  HoldTimeSet = 0;
  HoldOption = 0;
  HoldAlign = 0;
  MomentaryOverridePlus = 0;
  MomentaryOverrideMinus = 0;
  MomentaryOverrideZ = 0;

  SleepMode = 0;

  SettingTime = 0;  // Normally 0.
  // 1: hours, 2: minutes, 3: seconds, 4: seconds backward (stops ticking clock)

  AlignMode  = 0;  // Normally 0.
  OptionMode  = 0;  // Normally 0.
  AlignValue = 0;
  AlignRate = 2;
  AlignLoopCount = 0;
  StartingOption = 0;

  Wire.begin();

  /*
   // HIGHLY OPTIONAL: Set jardcoded RTC Time from within the program.
   // Example: Set time to 2:52:45.
   
   RTCsetTime(2,52,45);
   */

  ExtRTC = 0;
  UpdateRTC = 1;  // Force RTC update if there is one

  // Check if RTC is available, and use it to set the time if so.
  ExtRTC = RTCgetTime();
  // If no RTC is found, no attempt will be made to use it thereafter.

  if (ExtRTC)  // If time is already set from the RTC...
    VCRmode = 0;

// Uncomment to compute logarithmic brightness values for fade
//  for(int i=0; i <= 200; i++) {
//    FadeConv[i] = pow(i/200.0,fadeGamma)*63.0;
//  }

  cli();  // disable interrupts
// we'll use Timer1 to create an interrupt
  TCCR1A = 0;
  TCCR1B = (1<<WGM12);  // CTC mode with OCR1A as TOP
//  OCR1A = 319;  // 16000000/320 = 50,000 hz
  OCR1A = 159;  // 16000000/160 = 100,000 hz
//  OCR1A = 39;  // 16000000/8/40 = 20,000 hz
//  OCR1A = 24;  // 16000000/8/25 = 80,000 hz
//  OCR1A = 19;  // 16000000/8/20 = 100,000 hz
  TCCR1B |= (1<<CS10); // connect at 1x to start counter
//  TCCR1B |= (1<<CS11); // connect at clk/8 to start counter
  TCNT1 = 0;
  TIMSK1 |= (1<<OCIE1A); // enable Timer1 COMPA interrupt:
//  TCCR0A &= ~(_BV(WGM01) | _BV(WGM00));  // disable Timer0 PWM interrupts
  sei();

//  GPSinit(96);
//  gpsEnabled = true;
  
}  // End Setup

// Arduino Timer0 prescaler = 64; 16000000/64 = 250,000 hz
volatile uint8_t mpx_counter = 0;
volatile uint8_t mpx_limit = 100;  // call GPS Read at 1,000 hz
// Timer1 interrupt runs once every 0.025 ms  (40,000 hz)
SIGNAL(TIMER1_COMPA_vect) 
{
//  TCNT1 = 0;
  if (DisplayOn)
    DisplayMPX();
//  mpx_counter ++;
//  if (mpx_counter > mpx_limit) {
//    mpx_counter = 0;
//    if (gpsEnabled) GPSread();
//  }
}

#define count_max 254
#define select_max 8
byte mpx_count = 0;
byte mpx_select = 0;
void DisplayMPX(void)  // called at 0.025 ms intervals; does not loop
{
  mpx_count ++;
  switch(mpx_select)
  {
    case 0:
      if (mpx_count > D0)
        NextLED();
      break;
    case 1:
      if (mpx_count > D1)
        NextLED();
      break;
    case 2:
      if (mpx_count > D2)
        NextLED();
      break;
    case 3:
      if (mpx_count > D3)
        NextLED();
      break;
    case 4:
      if (mpx_count > D4)
        NextLED();
      break;
    case 5:
      if (mpx_count > D5)
        NextLED();
      break;
    case 6:
      if (mpx_count > ((8-MainBright)*20))
        NextLED();
      break;
    case 7:
      if (mpx_count > ((8-MainBright)*20))
        NextLED();
      break;
    case 8:
      if (mpx_count > ((8-MainBright)*20))
        NextLED();
      break;
  }
}

void NextLED(void)
{
  AllLEDsOff();
  mpx_count = 0;
  mpx_select ++;  // next LED
  if (mpx_select > select_max)
    mpx_select = 0;  // wrap back to first LED
  switch(mpx_select)  // now turn the LED on
  {
    case 0:
        TakeHigh(H0);
        TakeLow(L0);
      break;
    case 1:
        TakeHigh(H1);
        TakeLow(L1);
      break;
    case 2:
        TakeHigh(H2);
        TakeLow(L2);
      break;
    case 3:
        TakeHigh(H3);
        TakeLow(L3);
      break;
    case 4:
        TakeHigh(H4);
        TakeLow(L4);
      break;
    case 5:
        TakeHigh(H5);
        TakeLow(L5);
      break;
  }
}

void Blink(long time)  // blink the display to show something happened
{
  DisplayOn = false;
  AllLEDsOff();
  delay(time);
  DisplayOn = true;
}

void IncrAlignVal (void)
{
  AlignValue++;

  if (AlignMode < 5)  // seconds or minutes
  {
    if (AlignValue > 29)
      AlignValue = 0;
  }
  else
  {
    if (AlignValue > 11)
      AlignValue = 0;
  }
}


void DecrAlignVal (void)
{
  if (AlignValue > 0)
    AlignValue--;
  else if (AlignMode < 5)  // seconds or minutes
  {
    AlignValue = 29;
  }
  else  // hours
  {
    AlignValue = 11;
  }
}


void CheckButtons(void)
{
  byte PINDcopy;
  PINDcopy = PIND & buttonmask;

  if (PINDcopy != PINDLast)  // Button change detected
  {

    VCRmode = 0;  // End once any buttons have been pressed...
    TimeSinceButton = 0;

    if ((PINDcopy & 32) && ((PINDLast & 32) == 0))
    {  // "+" Button was pressed previously, and was just released!

      if ( MomentaryOverridePlus)
      {
        MomentaryOverridePlus = 0;
        // Ignore this transition if it was part of a hold sequence.
      }
      else
      {
        if (SleepMode)
          SleepMode = 0;
        else{
          if (AlignMode) {

            if ( AlignMode & 1)  // Odd mode:
            {
              if (AlignRate < 2)
                AlignRate++;
            }
            else
              IncrAlignVal();  // Even mode:

          }
          else if (OptionMode) {

            if (OptionMode == 1)
            {
              if (HourBright < 62)
                HourBright += 2;
            }
            if (OptionMode == 2)
            {
              if (MinBright < 62)
                MinBright += 2;
            }
            if (OptionMode == 3)
            {
              if (SecBright < 62)
                SecBright += 2;
            }
            if (OptionMode == 4)
            {
              CCW = 0;
            }
            if (OptionMode == 5)
            {
              if (FadeMode < FadeModes)
                FadeMode ++;
            }

          }
          else if (SettingTime) {
            
            if (SettingTime == 1)
              timeNow += 3600;
            if (SettingTime == 2)
              timeNow += 60;
            if (SettingTime > 2) { // could be 3 or 4
              timeNow ++;
              SettingTime = 3;  // allow clock to tick
            }
            
            if (timeNow > 43200)
              timeNow -= 43200;  // 12 hour time in seconds
           
          }
          else {
            // Brightness control mode
            MainBright++;
            if (MainBright > 8)
              MainBright = 8;  
          }
        }
      }
    }

    if ((PINDcopy & 64) && ((PINDLast & 64) == 0))
    {  // "-" Button was pressed and just released!

      VCRmode = 0;  // End once any buttons have been pressed...
      TimeSinceButton = 0;

      if ( MomentaryOverrideMinus)
      {
        MomentaryOverrideMinus = 0;
        // Ignore this transition if it was part of a hold sequence.
      }
      else
      {
        if (SleepMode)
          SleepMode = 0;
        else{
          if (AlignMode) {

            if ( AlignMode & 1)  // Odd mode:
            {
              if (AlignRate > -3)
                AlignRate--;
            }
            else
              DecrAlignVal();  // Even mode:

          }
          else if (OptionMode) {

            if (OptionMode == 1)
            {
              if (HourBright > 1)
                HourBright -= 2;
            }
            if (OptionMode == 2)
            {
              if (MinBright > 1)
                MinBright -= 2;
            }
            if (OptionMode == 3)
            {
              if (SecBright > 1)
                SecBright -= 2;
            }
            if (OptionMode == 4)
            {
              CCW = 1;
            }
            if (OptionMode == 5)
            {
              if (FadeMode > 0)
                FadeMode --;
            }

          }
          else if (SettingTime) {
            if (SettingTime == 1)
              timeNow -= 3600;
            if (SettingTime == 2)
              timeNow -= 60;
            if (SettingTime > 2) {  // could be 3 or 4
              timeNow --;
              SettingTime = 4;  // stop ticking if setting seconds back
            }
            
            if (timeNow < 0)
              timeNow += 43200;  // wrap

          }
          else {  // Normal brightness adjustment mode
            if (MainBright > 1) 
              MainBright--;
            else
              MainBright = 1;
          }
        }
      }
    }

    if ((PINDcopy & 128) && ((PINDLast & 128) == 0))
    {  // "Z" Button was pressed and just released!

      VCRmode = 0;  // End once any buttons have been pressed...
      TimeSinceButton = 0;

      if ( MomentaryOverrideZ)
      {
        MomentaryOverrideZ = 0;
        // Ignore this transition if it was part of a hold sequence.
      }
      else
      {
        if (AlignMode) {

          AlignMode++;
          if (AlignMode > 6)
            AlignMode = 1;

          AlignValue = 0;
          AlignRate = 2;
        }
        else if (OptionMode) {

          OptionMode++;
          StartingOption = 0;
          if (OptionMode > 5)
            OptionMode = 1;
          if (OptionMode > 3)  // CW vs CCW OR fade mode
            StartingOption = StartOptTimeLimit;  // don't go into loop

        }
        else if (SettingTime) {
          SettingTime++;
          if (SettingTime > 3)
            SettingTime = 1;
        }
        else {

          if (SleepMode == 0)
            SleepMode = 1;
          else
            SleepMode = 0;
        }
      }
    }
  }

  PINDLast = PINDcopy;
}


void CheckHeld(void)
{
    // Check to see if any buttons are being held down:

    if (( PIND & buttonmask) == buttonmask)
    {
      // No buttons are pressed.
      // Reset the variables that check to see if buttons are being held down.

      HoldTimeSet = 0;
      HoldOption = 0;
      HoldAlign = 0;
      FactoryResetDisable = 1;

      if (TimeSinceButton < 250)
        TimeSinceButton++;

      if (TimeSinceButton == 10)  // 10 s after last button released...
      {
        if (LastSavedBrightness != MainBright)
        {
          EESaveSettings();
        }
      }
    }
    else
    {

      // Note which buttons are being held down

      if (( PIND & buttonmask) == 128)  // "+" and "-" are pressed down. "Z" is up.
      {
        HoldAlign++;  // We are holding for alignment mode.
        HoldOption = 0;
        HoldTimeSet = 0;
      }
      if (( PIND & buttonmask) == 64)  // "+" and "Z" are pressed down. "-" is up.
      {
        HoldOption++;  // We are holding for option setting mode.
        HoldTimeSet = 0;
        HoldAlign = 0;
      }
      if (( PIND & buttonmask) == 96)  // "Z" is pressed down. "+" and "-" are up.
      {
        HoldTimeSet++;  // We are holding for time setting mode.
        HoldOption = 0;
        HoldAlign = 0;
      }
    }

    if (HoldAlign == 3)
    {
      MomentaryOverridePlus = 1;   // Override momentary-action of switches
      MomentaryOverrideMinus = 1;  // since we've detected a hold-down condition.

      OptionMode = 0;
      SettingTime = 0;

      // Hold + and - for 3 s AT POWER ON to restore factory settings.
      if ( FactoryResetDisable == 0){
        ApplyDefaults();
        EESaveSettings();
      }
      else
      {
        if (AlignMode) {
          AlignMode = 0;
        }
        else {
          AlignMode = 1;
          AlignValue = 0;
          AlignRate = 2;
        }
      }
    }

    if (HoldOption == 3)
    {
      MomentaryOverridePlus = 1;
      MomentaryOverrideZ = 1;
      AlignMode = 0;
      SettingTime = 0;

      if (OptionMode) {
        OptionMode = 0;
        EESaveSettings();  // Save options if exiting option mode!
      }
      else {
        OptionMode = 1;
        StartingOption = 0;
      }
    }

    if (HoldTimeSet == 3)
    {
      MomentaryOverrideZ = 1;

      if (AlignMode + OptionMode + SettingTime)  {
        // If we were in any of these modes, let's now return us to normalcy.
        // IF we are exiting time-setting mode, save the time to the RTC, if present:
        if (SettingTime && ExtRTC) {
//          RTCsetTime(HrNow,MinNow,SecNow);
          RTCsetTime(timeNow);
          Blink(200);  // Blink LEDs off to indicate RTC time set
        }

        if (OptionMode) {
          EESaveSettings();  // Save options if exiting option mode!
        }

        SettingTime = 0;
      }
      else
      {  // Go to setting mode IF and ONLY IF we were in regular-clock-display mode.
        SettingTime = 1;  // Start with HOURS in setting mode.
      }

      AlignMode = 0;
      OptionMode = 0;

    }
}


// ==============================  Main Loop  ============================== //
void loop()
{
  byte HighLine, LowLine;
  byte RefreshTime;

  RefreshTime = AlignMode + SettingTime + OptionMode;
  millisNow = millis();
//  millisNow += 1;  // temp - simulate 1 ms per 10 ms loop

  CheckButtons();

//  millisNow = millis();
  // Since millisNow & millisThen are both unsigned long, this will work correctly even when millis() wraps
  if ((millisNow - millisThen) >= 1000)  // has 1 second gone by?
  {
    millisThen += 1000;  // do this again in 1 second

    CheckHeld();
    
    if (SettingTime < 4) { // if not setting seconds back
      timeNow++;  // the clock ticks...
      if (timeNow>43200)
       timeNow -= 43200;
      RefreshTime = 1;
    }
    
    if (((timeNow%60) == 0) && (SettingTime == 0) && ExtRTC)  // Check RTC once per minute, if not setting time & RTC enabled
      RTCgetTime();

  }
  

  if (RefreshTime) {
    // Calculate which LEDs to light up to give the correct shadows:

    if (AlignMode) {
      AlignDisplay();
    }
    else if (OptionMode) {  // Option setting mode
      OptionDisplay();
    }
    else    {  // Regular clock display
      NormalTimeDisplay();
    }

    LH1 = HrDisp;
    LH2 = HrNext;
    LM1 = MinDisp;
    LM2 = MinNext;
    LS1 = SecDisp;
    LS2 = SecNext;

    if (CCW){
      // Counterclockwise
      if (HrDisp)
        LH1 = 12 - HrDisp;
      if (HrNext)
        LH2 = 12 - HrNext;
      if (MinDisp)
        LM1 = 30 - MinDisp;
      if (MinNext)
        LM2 = 30 - MinNext;
      if (SecDisp)
        LS1 = 30 - SecDisp;
      if (SecNext)
        LS2 = 30 - SecNext;

      // Serial.print(HrDisp,DEC);
      // Serial.print(", ");
      // Serial.println(H3,DEC);
    }  // if (CCW)

  }

  SecFade1 = 63;  // 1st LED bright
  SecFade2 = 0;  // 2nd LED dim
  MinFade1 = 63;
  MinFade2 = 0;
  HrFade1 = 63;
  HrFade2 = 0;

  if (SettingTime)  // i.e., if (SettingTime is nonzero)
  {

    HrFade1 = 5;  // make them all dim
    MinFade1 = 5;
    SecFade1 = 5;

    if (SettingTime == 1)  // hours
    {
      HrFade1  = fadeMax;  // make hours bright
    }
    if (SettingTime == 2)  // minutes
    {
      MinFade1  = fadeMax;  // make minutes bright
      if (timeNow/60 & 1)  // odd minutes 
        MinFade2 = fadeMax;  // 2nd LED on as well (wbp)
    }
    if (SettingTime > 2)  // seconds
    {
      SecFade1 = fadeMax;  // make seconds bright
      if (timeNow & 1)  // odd seconds
        SecFade2 = fadeMax;  // 2nd LED on as well (wbp)
    }

  }
  else if (AlignMode + OptionMode)  // if either...
  {
    HrFade1 = 0;
    MinFade1 = 0;
    SecFade1 = 0;

    if (AlignMode){
      if (AlignMode < 3)
        SecFade1 = fadeMax;
      else if (AlignMode > 4)
        HrFade1 = fadeMax;
      else
        MinFade1 = fadeMax;
    }
    else {  // Must be OptionMode....
      if (StartingOption < StartOptTimeLimit)
      {
        if (OptionMode == 1)
        {
          HrFade1 = fadeMax;
        }
        if (OptionMode == 2)
        {
          MinFade1 = fadeMax;
        }
        if (OptionMode == 3)
        {
          SecFade1 = fadeMax;
        }
        if (OptionMode == 4)  // CW vs CCW
        {
          SecFade1 = fadeMax;
          MinFade1 = fadeMax;
        }
      }
      else {  // No longer in starting mode.

        HrFade1 = fadeMax;
        MinFade1 = fadeMax;
        SecFade1 = fadeMax;

        if (OptionMode == 4)  // CW vs CCW
        {
          HrFade1 = 0;
        }
        else
          NormalFades();
      }
    }

  }
  else
  {
    NormalFades();
  }

  if (SleepMode || (VCRmode && (timeNow & 1))) {
    DisplayOn = false;
    AllLEDsOff();
  }
  else
    DisplayOn = true;

  cli();
  if (RefreshTime) {
//    cli();
    L0 = HrLo[LH1];
    H0 = HrHi[LH1];
    L1 = HrLo[LH2];
    H1 = HrHi[LH2];
    L2 = MinLo[LM1];
    H2 = MinHi[LM1];
    L3 = MinLo[LM2];
    H3 = MinHi[LM2];
    L4 = SecLo[LS1];
    H4 = SecHi[LS1];
    L5 = SecLo[LS2];
    H5 = SecHi[LS2];
//    sei();
  }

// set brightness for each of 6 LED's
// MainBright = 1 to 8
// xxFaden = 0 to 63
// xxBright = 1 to 63 ???
// dn = 63*63*8/128 = 0 to 248
//  cli();
  byte of = 1;
  D0 = HourBright*HrFade1*MainBright >> 7 + of;  // hbrt * fade * brt / 128
  D1 = HourBright*HrFade2*MainBright >> 7 + of;  // the "+1" eliminates small glitches at top of second
  D2 = MinBright*MinFade1*MainBright >> 7 + of;
  D3 = MinBright*MinFade2*MainBright >> 7 + of;
  D4 = SecBright*SecFade1*MainBright >> 7 + of;
  D5 = SecBright*SecFade2*MainBright >> 7 + of;
  sei();


  /*
  temp = millis() - temp;
  Serial.println(temp,DEC);
   */

  // Can this sync be tried only once per second?
  if( getPCtime()) {  // try to get time sync from pc
    adjustTime();
  }

//  if (gpsEnabled) {
//    if (gpsDataReady()) {
//      parseGPSdata(gpsNMEA());  // get the GPS serial stream and possibly update the clock 
//    }
//  }

  while ((millis() - millisNow) < 10) {  // run the main loop at 100 hz 
//    asm("nop");
    if (Serial.available())  // data on the serial port?
      if (getGPSdata()) { // collect GPS data and parse it, maybe set the time
        adjustTime();
      }
  }

}  // END Loop

