/**
 * \file 
 * \brief RTC Driver
 *
 * The RTC Driver gets times, alarms, Unique ID from a MPC79411 via I2C.
 */
#ifndef _RTC_H
#define _RTC_H
/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include <Cpu/Std/Ifx_Types.h>

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
// this is for our time from the RTC (BCD coded)
typedef volatile struct
{
  uint8 seconds;
  uint8 minutes;
  uint8 hours;
  uint8 day;
  uint8 date;
  uint8 month;
  uint8 year;
  volatile union {
    uint8 U;
    struct {
      uint8 running :1;   // when set then the rtc is running or should be run
      uint8 hour_12 :1;   // when set we use 12 hours time, when clear the 24 hours
      uint8 pm_set  :1;   // when set it is pm, only when we use 12 hours time (hour_12 is set)
      uint8 leap_year :1; // when set then we have a leap year, read only
      uint8 alarm0  :1;   // when set then alarm 0 is activ
      uint8 alarm1  :1;   // when set then alarm 1 is activ
      uint8 reserved :2;  // reserved bits
    } B;
  } flags;
} TIME;

static inline uint8 Bcd2Dec(uint8 bcd)
{
  return ((bcd >> 4)*10+(bcd & 0x0F));
}

static inline uint8 Dec2Bcd(uint8 dec){
  return (((dec/10)<<4)+(dec%10));
}
/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/


sint32 rtc_init (void);
sint32 rtc_gettime (TIME *pActTime);
sint32 rtc_getalarm (TIME *pActTime, uint8 uiAlarmNumber);
sint32 rtc_settime (TIME *pActTime);
sint32 rtc_setalarm (TIME *pAlarmTime, uint8 uiAlarmNumber, uint8 uiAlarmMatch);
sint32 rtc_alarm_onoff (uint8 uiAlarmNumber, uint8 uiOn);
sint32 rtc_reset_alarmflag (uint8 uiAlarmNumber);
sint32 rtc_calibration (void);
sint32 rtc_get_unique_id (uint8 *pUniqueId);

#endif /* _RTC_H */
