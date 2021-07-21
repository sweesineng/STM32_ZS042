/**
  ******************************************************************************
  * @file    at24cxx.h
  * @brief   This file contains all the constants parameters for the AT24C32/64
  *          EEPROM
  * @Date	 21 Jul 2021
  ******************************************************************************
  * @attention
  * Usage:
  *		Uncomment LL Driver for HAL driver
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AT24CXX_H
#define AT24CXX_H

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

/* AT24Cxx Register ----------------------------------------------------------*/
#define AT24Cxx_ADDRESS 		0xAE	// ZS-042 AT24C32 default address
#define AT24Cxx_EEPROM_SIZE		0x1000	// EEPROM Size 4096 byte
#define AT24Cxx_PAGE_SIZE		0x20	// Page size 32 byte
#define AT24Cxx_PAGE_NUM		0x80	// Number of page 128

/* AT24Cxx External Function -------------------------------------------------*/
uint8_t AT24Cxx_EraseChip(void);
uint8_t AT24Cxx_FillPage(uint16_t Page, uint8_t Val);
uint8_t AT24Cxx_ReadByte(uint16_t MemAddr, uint8_t *pData, uint16_t Len);
uint8_t AT24Cxx_WriteByte(uint16_t MemAddr, uint8_t *value, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif	/* AT24CXX_H */
