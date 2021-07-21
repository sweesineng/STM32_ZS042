/**
  ******************************************************************************
  * @file    ds3231.h
  * @brief   This file contains all the constants parameters for the DS3231 RTC
  * @Date	 21 Jul 2021
  ******************************************************************************
  * @attention
  * Usage:
  *		Uncomment LL Driver for HAL driver
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DS3231_H
#define DS3231_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"

/* Driver Selection ----------------------------------------------------------*/
//#define LL_Driver

#ifdef LL_Driver
#define I2Cx					I2C1
#else
#define I2Cx					(&hi2c1)
#endif

/* Custom Define -------------------------------------------------------------*/
#define ALARM_DYDT				0x00	// Logic 0 date(DOM), Logic 1 day(DOW)

/* DS3231 Register -----------------------------------------------------------*/
#define DS3231_Disable			0x00
#define DS3231_Enabled			0x01

#define DS3231_ADDRESS 			0xD0
#define DS3231_ALARM_1			0X07
#define DS3231_ALARM_2			0X0B
#define DS3231_TEMP_MSB			0X11
#define DS3231_CONTROL			0X0E
#define DS3231_STATUS			0X0F

#define DS3231_CTRL_A1IE		0x00	// Alarm 1 Interrupt Enable
#define DS3231_CTRL_A2IE		0x01	// Alarm 2 Interrupt Enable
#define DS3231_CTRL_INTCN		0x02	// Interrupt Control
#define DS3231_CTRL_RS1			0x03	// Rate Select 2
#define DS3231_CTRL_RS2			0x04	// Rate Select 2
#define DS3231_CTRL_CONV		0x05	// Convert Temperature
#define DS3231_CTRL_BBSQW		0x06	// Battery-Backed Square-Wave Enable
#define DS3231_CTRL_EOSC		0x07	// Enable Oscillator

#define DS3231_STAT_A1F			0x00	// Alarm 1 Flag
#define DS3231_STAT_A2F			0x01	// Alarm 2 Flag
#define DS3231_STAT_BSY			0x02	// Busy
#define DS3231_STAT_EN32K		0x03	// Enable 32kHz Output
#define DS3231_STAT_OSF			0x07	// Oscillator Stop Flag

/* Custom Macro --------------------------------------------------------------*/
#define DecToBcd(val) ( (val/10*16) + (val % 10) )
#define BcdToDec(val) ( (val/16*10) + (val % 16) )

/* DS3231 Data Structure -----------------------------------------------------*/
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME_Handle;

typedef enum
{
  MODE_ALL,
  MODE_HOUR_MIN_SEC,
  MODE_HOUR_MIN,
  MODE_MIN_SEC,
  MODE_MIN,
  MODE_SEC,
  MODE_ONCE_PER_MIN,
  MODE_ONCE_PER_SEC
} AlarmMode;

typedef enum
{
	SQW_1HZ,
	SQW_1024HZ,
	SQW_4096HZ,
	SQW_8192HZ
}SQWRate;

typedef enum
{
	ALL,
	A1,
	A2
}Alarm;

/* DS3231 External Function --------------------------------------------------*/
float DS3231_GetTemp (void);
void DS3231_TimeGet(TIME_Handle *Now);
void DS3231_TimeSet(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow,
		uint8_t dom, uint8_t month, uint8_t year);
void DS3231_SetAlarm1(uint8_t mode, uint8_t sec, uint8_t min, uint8_t hour,
		uint8_t dydt);
void DS3231_SetAlarm2(uint8_t mode, uint8_t min, uint8_t hour, uint8_t dydt);
void DS3231_EnableConvTemp(uint8_t mode);
void DS3231_ClearAlarm(uint8_t Alarm);
void DS3231_OscillatorEOSC(uint8_t mode);
void DS3231_32kHzOutput(uint8_t mode);
void DS3231_EnableInterrupt(uint8_t mode);
void DS3231_Alarm1_Interrupt(uint8_t mode);
void DS3231_Alarm2_Interrupt(uint8_t mode);
void DS3231_BaterryBackedSQW(uint8_t mode);
void DS3231_SetSQWFreq(SQWRate freq);
uint8_t DS3231_GetFlag(uint8_t Reg);
void DS3231_GetAlarm(void);

#ifdef __cplusplus
}
#endif

#endif	/* DS3231_H */
