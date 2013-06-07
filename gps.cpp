/*
 * GPS support for Alpha Clock Five
 * (C) 2013 William B Phelps
 * All commercial rights reserved
 */

#include "gps.h"

#ifndef DST_offset
uint8_t DST_offset = 1;
#endif

uint8_t GPS_mode = 0;  // 0, 48, 96
#define gpsTimeoutLimit 5  // 5 seconds until we display the "no gps" message

unsigned long tGPSupdateUT = 0;  // time since last GPS update in UT
unsigned long tGPSupdate = 0;  // time since last GPS update in local time
byte GPSupdating = false;

int8_t TZ_hour = -8;
int8_t TZ_minutes = 0;
//uint8_t DST_offset = 0;

// GPS parser for 406a
#define GPSBUFFERSIZE 128 // plenty big
char gpsBuffer[GPSBUFFERSIZE];

// get data from gps and update the clock (if possible)
byte getGPSdata(void) {
//  char charReceived = UDR0;  // get a byte from the port
  byte rc = 0;  // return 1 if system time set from GPS time
  char charReceived = Serial.read();
  uint8_t bufflen = strlen(gpsBuffer);
  //If the buffer has not been started, check for '$'
  if ( ( bufflen == 0 ) &&  ( '$' != charReceived ) )
    return rc;  // wait for start of next sentence from GPS
  if ( bufflen < (GPSBUFFERSIZE - 1) ) {  // is there room left? (allow room for null term)
    if ( '\r' != charReceived ) {  // end of sentence?
      strncat(gpsBuffer, &charReceived, 1);  // add char to buffer
      return rc;
    }
    strncat(gpsBuffer, "*", 1);  // mark end of buffer just in case
    //beep(1000, 1);  // debugging
    // end of sentence - is this the message we are looking for?
    if (strncmp(gpsBuffer, "$GPRMC", 6) == 0)
      rc = parseGPSdata(gpsBuffer);  // check for GPRMC sentence and set clock
  }  // if space left in buffer
  // either buffer is full, or the message has been processed. reset buffer for next message
  memset( gpsBuffer, 0, GPSBUFFERSIZE );  // clear GPS buffer
  return rc;
}  // getGPSdata

char status;
char checksum[3];
char gpsTime[11];
uint16_t gps_cks_errors, gps_parse_errors, gps_time_errors;

uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  while (str[0] != 0) {
    if ((str[0] > '9') || (str[0] < '0'))
      return d;  // no more digits
    d = (d*10) + (str[0] - '0');
    str++;
  }
  return d;
}
const char hex[17] = "0123456789ABCDEF";
uint8_t atoh(char x) {
  return (strchr(hex, x) - hex);
}
uint32_t hex2i(char *str, uint8_t len) {
  uint32_t d = 0;
  for (uint8_t i=0; i<len; i++) {
    d = (d*10) + (strchr(hex, str[i]) - hex);
  }
  return d;
}
//  225446       Time of fix 22:54:46 UTC
//  A            Navigation receiver warning A = OK, V = warning
//  4916.45,N    Latitude 49 deg. 16.45 min North
//  12311.12,W   Longitude 123 deg. 11.12 min West
//  000.5        Speed over ground, Knots
//  054.7        Course Made Good, True
//  191194       Date of fix  19 November 1994
//  020.3,E      Magnetic variation 20.3 deg East
//  *68          mandatory checksum

//$GPRMC,050942.00,A,3726.31813,N,12207.44407,W,0.199,,070613,,,A*67\r  (VK-162)
//$GPRMC,225446.000,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n
// 0         1         2         3         4         5         6         7
// 0123456789012345678901234567890123456789012345678901234567890123456789012
//    0     1       2    3    4     5    6   7     8      9     10  11 12
byte parseGPSdata(char *gpsBuffer) {
  byte rc = 0;  // return 1 if system time updated from GPS time
  time_t tNow, tDelta;
  tmElements_t tm;
  uint8_t gpsCheck1, gpsCheck2;  // checksums
  uint8_t errno = 0;
  //	char gpsTime[10];  // time including fraction hhmmss.fff
  char gpsFixStat;  // fix status
  //	char gpsLat[7];  // ddmm.ff  (with decimal point)
  //	char gpsLatH;  // hemisphere 
  //	char gpsLong[8];  // dddmm.ff  (with decimal point)
  //	char gpsLongH;  // hemisphere 
  //	char gpsSpeed[5];  // speed over ground
  //	char gpsCourse[5];  // Course
  //	char gpsDate[6];  // Date
  //	char gpsMagV[5];  // Magnetic variation 
  //	char gpsMagD;  // Mag var E/W
  //	char gpsCKS[2];  // Checksum without asterisk
  char *ptr, *ptr2;
  uint32_t tmp;
  if ( strncmp( gpsBuffer, "$GPRMC,", 7 ) == 0 ) {  
    Serial.println(gpsBuffer);
    //beep(1000, 1);
    //Calculate checksum from the received data
    ptr = &gpsBuffer[1];  // start at the "G"
    gpsCheck1 = 0;  // init collector
    /* Loop through entire string, XORing each character to the next */
    while (*ptr != '*') // count all the bytes up to the asterisk
    {
      gpsCheck1 ^= *ptr;
      ptr++;
      if (ptr>(gpsBuffer+GPSBUFFERSIZE)) goto GPSerror1;  // extra sanity check, can't hurt...
    }
    // now get the checksum from the string itself, which is in hex
    gpsCheck2 = atoh(*(ptr+1)) * 16 + atoh(*(ptr+2));
    if (gpsCheck1 == gpsCheck2) {  // if checksums match, process the data
      //beep(1000, 1);
      ptr = gpsBuffer+6;  // first comma after "$GPRMC" (,hhmmss.xxx)
//      errno=1;
//      if (ptr == NULL) goto GPSerror1;
      ptr2 = strchr(ptr+1, ',');  // find the comma after Time
      errno=2;
      if (ptr2 == NULL) goto GPSerror1;
      errno=3;
      if (((ptr2-ptr) < 6) || ((ptr2-ptr) > 10)) goto GPSerror1;  // check time length
      tmp = parsedecimal(ptr+1);   // parse integer portion
      tm.Hour = tmp / 10000;
      tm.Minute = (tmp / 100) % 100;
      tm.Second = tmp % 100;
      ptr = ptr2;  // ,Status
      errno=4;
      if (ptr == NULL) goto GPSerror1;
      gpsFixStat = ptr[1];  // ,A,3726.31813,N,12207.44407,W,0.199,,070613,,,A*67\r
//      if (gpsFixStat == 'A') {  // if data valid, parse time & date
        ptr = strchr(ptr+1, ',');  // ,Latitude including fraction ,3726.31813,N,12207.44407,W,0.199,,070613,,,A*67\r
        errno=5;
        if (ptr == NULL) goto GPSerror1;
        //  strncpy(gpsLat, ptr, 7);  // copy Latitude ddmm.ff
        ptr = strchr(ptr+1, ',');  // ,Latitude N/S
        errno=6;
        if (ptr == NULL) goto GPSerror1;
        //  gpsLatH = ptr[0];
        ptr = strchr(ptr+1, ',');  // ,Longitude including fraction ,12207.44407,W,0.199,,070613,,,A*67\r
        errno=7;
        if (ptr == NULL) goto GPSerror1;
        //  strncpy(gpsLong, ptr, 7);
        ptr = strchr(ptr+1, ',');  // ,Longitude Hemisphere
        errno=8;
        if (ptr == NULL) goto GPSerror1;
        //  gpsLongH = ptr[0];
        ptr = strchr(ptr+1, ',');  // ,Ground speed ,0.199,,070613,,,A*67\r
        errno=9;
        if (ptr == NULL) goto GPSerror1;
        //  strncpy(gpsSpeed, ptr, 5);
        ptr = strchr(ptr+1, ',');  // ,Track angle (course) ,,070613,,,A*67\r
        errno=10;
        if (ptr == NULL) goto GPSerror1;
        //  strncpy(gpsCourse, ptr, 5);
        ptr = strchr(ptr+1, ',');  // ,ddmmyy ,070613,,,A*67\r
        errno=11;
        if (ptr == NULL) goto GPSerror1;
        //  strncpy(gpsDate, ptr, 6);
        ptr2 = strchr(ptr+1, ',');  // comma after date ,,,A*67\r
        errno=12;
        if ((ptr2-ptr) != 7) goto GPSerror1;  // check date length
        tmp = parsedecimal(ptr+1);
        tm.Day = tmp / 10000;
        tm.Month = (tmp / 100) % 100;
        tm.Year = tmp % 100;
        ptr = strchr(ptr+1, ',');  // magnetic variation ,,A*67\r
        errno=13;
        if (ptr == NULL) goto GPSerror1;
        ptr = strchr(ptr+1, ',');  // magnetic var direction ,A*67\r
        errno=14;
        if (ptr == NULL) goto GPSerror1;
        // That's all - we already checked for \r and the checksum has been checked

        tm.Year = y2kYearToTm(tm.Year);  // convert yy year to (yyyy-1970) (add 30)
        tNow = makeTime(tm);  // convert to time_t
        tDelta = abs(tNow-tGPSupdateUT);

        if ((tGPSupdateUT > 0) && (tDelta > SECS_PER_DAY))  goto GPSerror2;  // GPS time jumped more than 1 day
        GPSupdating = false;  // valid GPS data received, flip the LED off

        if ((tm.Second == 30) || (tDelta >= 60)) {  // update RTC once/minute or if it's been 60 seconds
          //beep(1000, 1);  // debugging
//          a5loadOSB_DP("____2",a5_brightLevel);  // wbp
//          a5BeginFadeToOSB();  
          GPSupdating = true;
          tGPSupdateUT = tNow;  // remember time of this update (UT)
          tNow = tNow + (long)(TZ_hour + DST_offset) * SECS_PER_HOUR;  // add time zone hour offset & DST offset
          if (TZ_hour < 0)  // add or subtract time zone minute offset
            tNow = tNow - (long)TZ_minutes * SECS_PER_MIN;  // 01feb13/wbp
          else
            tNow = tNow + (long)TZ_minutes * SECS_PER_MIN;  // 01feb13/wbp
          tGPSupdate = tNow;  // remember time of this update (local time)
          setTime(tNow);  // adjust system time
          Serial.print("time set from GPS to ");
          Serial.println(tNow, DEC);
          return 1;
        }

//      } // if fix status is A
    } // if checksums match
    else  // checksums do not match
    Serial.println("GPS CKS error");
    gps_cks_errors++;  // increment error count
    return 0;
GPSerror1:
    Serial.print("GPS Parse error (");
    Serial.print(errno);
    Serial.println(")");
    gps_parse_errors++;  // increment error count
    goto GPSerror2a;
GPSerror2:
    Serial.println("GPS Time error");
    gps_time_errors++;  // increment error count
GPSerror2a:
//    beep(2093,1);  // error signal - I'm leaving this in for now /wm
//    a5tone(2093,200);
//???    flash_display(200);  // flash display to show GPS error
    strcpy(gpsBuffer, "");  // wipe GPS buffer
    return 0;
  }  // if "$GPRMC"
}

