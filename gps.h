/*
 * GPS support for VFD Modular Clock
 * (C) 2012 William B Phelps
 *
 */

#if not defined GPS_H_
#define GPS_H_

#include "Time.h"
#include "Arduino.h"

// String buffer size:
//#define GPSBUFFERSIZE 96
//The year the clock was programmed, used for error checking
//#define PROGRAMMING_YEAR 12

extern int8_t TZ_hour;
extern int8_t TZ_minutes;
extern uint8_t DST_offset;

extern byte GPS_mode;
extern unsigned long tGPSupdate;
extern byte GPSupdating;

extern byte UseRTC;
extern unsigned long tGPSupdateUT;

//GPS serial data handling functions:
extern uint8_t gpsEnabled;
//uint8_t gpsDataReady(void);
//void GPSread(void);
//char *gpsNMEA(void);
byte getGPSdata(void);
byte parseGPSdata(char *gpsBuffer);

uint8_t leapyear(uint16_t y);
//void uart_init(uint16_t BRR);
void GPSinit(uint8_t gps);

#if (F_CPU == 16000000)
#define BRRL_4800 207    // for 16MHZ
#define BRRL_9600 103    // for 16MHZ
#define BRRL_192 52    // for 16MHZ
#elif (F_CPU == 8000000)
#define BRRL_4800 103
#define BRRL_9600 52
#define BRRL_192 26
#endif

#endif // GPS_H_

