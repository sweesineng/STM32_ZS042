/**
  ******************************************************************************
  * @file    at24cxx.c
  * @brief   This file includes the HAL/LL driver for DS3231 RTC
  ******************************************************************************
  */
#include "DS3231.h"

uint8_t Buffer[7];

/**
  * @brief  I2C Bus Write
  * @retval Success = 0, Failed = 1
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be Write
  */
uint8_t DS3231_Bus_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData,
		uint16_t Len) {
#ifdef LL_Driver
	uint16_t XferCount = Len;
	uint8_t *TxBuffer = pData;
	uint16_t XferSize = 0;

	/* Wait for I2C bus is free */
	while (LL_I2C_IsActiveFlag_BUSY(I2Cx)) {};

	if (!LL_I2C_IsEnabled(I2Cx))
	{
		LL_I2C_Enable(I2Cx);
	}

    /* Send Slave Address and Memory Address */
    LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT, 1,
    		I2C_CR2_RELOAD, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)){};

	 /* Send Memory Address */
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(MemAddr & (uint16_t)0x00FF)));
	while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (XferCount > 255U)
    {
        XferSize = 255U;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_RELOAD , LL_I2C_GENERATE_NOSTARTSTOP);
    }
    else
    {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_AUTOEND , LL_I2C_GENERATE_NOSTARTSTOP);
    }

    do
    {
        /* Wait until TXIS flag is set */
    	while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};

        /* Write data to TXDR */
        LL_I2C_TransmitData8(I2Cx, *TxBuffer);

        /* Increment Buffer pointer */
        TxBuffer++;

        XferCount--;
        XferSize--;

        if ((XferCount != 0U) && (XferSize == 0U))
        {
            /* Wait until TCR flag is set */
        	while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

            if (XferCount > 255U)
            {
                XferSize = 255U;
                LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
                		(uint8_t)XferSize, I2C_CR2_RELOAD ,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
            else
            {
                XferSize = XferCount;
                LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
                		(uint8_t)XferSize, I2C_CR2_AUTOEND ,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
        }
    } while (XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {};

    /* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2Cx);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Clear Configuration Register 2 */
    I2Cx->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
#else
	if (HAL_I2C_Mem_Write(I2Cx, DevAddr, MemAddr, I2C_MEMADD_SIZE_8BIT, pData,
			Len, HAL_MAX_DELAY) != 0)
	{
		return 1;
	}
#endif

	return 0;
}

/**
  * @brief  I2C Bus Read
  * @retval Success = 0, Failed = 1
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t DS3231_Bus_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData,
		uint16_t Len) {
#ifdef LL_Driver
    uint16_t XferCount = Len;
    uint16_t XferSize = 0;

	/* Wait for I2C bus is free */
	while (LL_I2C_IsActiveFlag_BUSY(I2Cx)) {};

	if (!LL_I2C_IsEnabled(I2Cx))
	{
		LL_I2C_Enable(I2Cx);
	}

    /* Send Slave Address and Memory Address */
    LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT, 1,
    		LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};

	 /* Send Memory Address */
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(MemAddr & (uint16_t)0x00FF)));
    while (!LL_I2C_IsActiveFlag_TC(I2Cx)) {};

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (XferCount > 255U)
    {
        XferSize = 255U;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_RELOAD, LL_I2C_GENERATE_START_READ);
    }
    else
    {
        XferSize = XferCount;
        LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT,
        		(uint8_t)XferSize, I2C_CR2_AUTOEND, LL_I2C_GENERATE_START_READ);
    }

    do
    {
        /* Wait until RXNE flag is set */
        while (!LL_I2C_IsActiveFlag_RXNE(I2Cx)) {};

        /* Read data from RXDR */
        *pData = LL_I2C_ReceiveData8(I2Cx);

        /* Increment Buffer pointer */
        pData++;
        XferCount--;
        XferSize--;

        if ((XferCount != 0U) && (XferSize == 0U))
        {
            /* Wait until TCR flag is set */
            while (!LL_I2C_IsActiveFlag_TCR(I2Cx)) {};

            if (XferCount > 255U)
            {
                XferSize = 255U;

                LL_I2C_HandleTransfer(I2Cx, DevAddr,
                		LL_I2C_ADDRSLAVE_7BIT, (uint8_t)XferSize, I2C_CR2_RELOAD,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
            else
            {
                XferSize = XferCount;
                LL_I2C_HandleTransfer(I2Cx, DevAddr,
                		LL_I2C_ADDRSLAVE_7BIT, (uint8_t)XferSize, I2C_CR2_AUTOEND,
						LL_I2C_GENERATE_NOSTARTSTOP);
            }
        }
    } while (XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically
     * generated.
     * Wait until STOPF flag is reset */
    while (!LL_I2C_IsActiveFlag_STOP(I2Cx)) {};

	/* Clear NACKF Flag */
    LL_I2C_ClearFlag_NACK(I2Cx);

    /* Clear STOP Flag */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Clear Configuration Register 2 */
    I2Cx->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R |
    		I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN));
#else
	if (HAL_I2C_Mem_Read(I2Cx, DevAddr, MemAddr, I2C_MEMADD_SIZE_8BIT, pData,
			Len, HAL_MAX_DELAY) != 0)
	{
		return 1;
	}
#endif

	return 0;
}

/**
  * @brief  Set Target time to RTC
  * @param  sec  	Seconds, 0-60
  * @param  min  	Minute, 0-60
  * @param  hour 	Hours, 0-23
  * @param  dow  	Days of the week, 1-7
  * @param  dom  	Days of the month, 1-31
  * @param  month  	Month, 1-12
  * @param  year  	Year, 21, 22 ...
  */
void DS3231_TimeSet (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom,
		uint8_t month, uint8_t year)
{
	memset(Buffer, 0x00, sizeof(Buffer));
	Buffer[0] = DecToBcd(sec);
	Buffer[1] = DecToBcd(min);
	Buffer[2] = DecToBcd(hour);
	Buffer[3] = DecToBcd(dow);
	Buffer[4] = DecToBcd(dom);
	Buffer[5] = DecToBcd(month);
	Buffer[6] = DecToBcd(year);

	DS3231_Bus_Write(DS3231_ADDRESS, 0x00, Buffer, sizeof(Buffer));
}

/**
  * @brief  Get Current time from RTC
  * @param  Now	Pointer to TIME_handler structure
  */
void DS3231_TimeGet (TIME_Handle *Now)
{
	DS3231_Bus_Read(DS3231_ADDRESS, 0x00, Buffer, 7);
	Now->seconds	= BcdToDec(Buffer[0]);
	Now->minutes 	= BcdToDec(Buffer[1]);
	Now->hour 		= BcdToDec(Buffer[2]);
	Now->dayofweek 	= BcdToDec(Buffer[3]);
	Now->dayofmonth = BcdToDec(Buffer[4]);
	Now->month 		= BcdToDec(Buffer[5]);
	Now->year 		= BcdToDec(Buffer[6]);
}

/**
  * @brief  Get Current temperature
  * @retval Temperate degree C in float
  */
float DS3231_GetTemp(void)
{
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_TEMP_MSB, Buffer, 2);
	return ((Buffer[0]) + (Buffer[1] >> 6) / 4.0f);
}

/**
  * @brief  Get Status Register flag
  * @param  Reg  Register
  * @retval Flag status in 0 or 1
  */
uint8_t DS3231_GetFlag(uint8_t Reg)
{
	uint8_t status = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	return ((status >> Reg) & 0x01);
}

/**
  * @brief  Enable Oscillator
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_OscillatorEOSC(uint8_t mode)
{
	uint8_t ctrl = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(1 << DS3231_CTRL_EOSC);	// Clearing bit
	ctrl |= (mode << DS3231_CTRL_EOSC);	// Setting bit
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}

/**
  * @brief  Enable Battery-Backed Square-Wave
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_BaterryBackedSQW(uint8_t mode)
{
	uint8_t ctrl = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(1 << DS3231_CTRL_BBSQW);
	ctrl |= (mode << DS3231_CTRL_BBSQW);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}

/**
  * @brief  Enable Alarm Interrupt
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_EnableInterrupt(uint8_t mode)
{
	uint8_t ctrl = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(1 << DS3231_CTRL_INTCN);
	ctrl |= (mode << DS3231_CTRL_INTCN);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}

/**
  * @brief  Enable Alarm1 Interrupt
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_Alarm1_Interrupt(uint8_t mode)
{
	uint8_t ctrl = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(1 << DS3231_CTRL_A1IE);
	ctrl |= (mode << DS3231_CTRL_A1IE);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}

/**
  * @brief  Enable Alarm2 Interrupt
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_Alarm2_Interrupt(uint8_t mode)
{
	uint8_t ctrl = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(1 << DS3231_CTRL_A2IE);
	ctrl |= (mode << DS3231_CTRL_A2IE);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}

/**
  * @brief  Clear ALL or Selected Alarm
  * @param  Alarm  Select Alarm, ALL, A1, A2
  */
void DS3231_ClearAlarm(Alarm Alarm)
{
	uint8_t status = 0x00;

	switch(Alarm)
	{
	case A1:
		/* Clear Control Register */
		DS3231_Alarm1_Interrupt(DS3231_Disable);
		/* Clear Status Register */
		DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
		status &= ~(1 << DS3231_STAT_A1F);
		status |= (0 << DS3231_STAT_A1F);
		DS3231_Bus_Write(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
		break;
	case A2:
		/* Clear Control Register */
		DS3231_Alarm2_Interrupt(DS3231_Disable);
		/* Clear Status Register */
		DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
		status &= ~(1 << DS3231_STAT_A2F);
		status |= (0 << DS3231_STAT_A2F);
		DS3231_Bus_Write(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	case ALL:
		/* Clear Control Register */
		DS3231_Alarm1_Interrupt(DS3231_Disable);
		DS3231_Alarm2_Interrupt(DS3231_Disable);
		/* Clear Status Register */
		DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
		status &= ~((1 << DS3231_STAT_A2F) | (1 << DS3231_STAT_A1F));
		status |= ((0 << DS3231_STAT_A2F) | (0 << DS3231_STAT_A2F));
		DS3231_Bus_Write(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	}
}

/**
  * @brief  Set Alarm 1 on RTC
  * @param  mode  	Matching mode, MODE_ALL, MODE_HOUR_MIN_SEC, MODE_MIN_SEC,
  * 				MODE_SEC, MODE_ONCE_PER_SEC
  * @param  sec  	Seconds, 0-60
  * @param  min  	Minute, 0-60
  * @param  hour 	Hours, 0-23
  * @param  day  	Date or Day
  */
void DS3231_SetAlarm1(uint8_t mode, uint8_t sec, uint8_t min, uint8_t hour,
		uint8_t dydt)
{
	uint8_t status = 0x00;
	uint8_t Target[4];
	Target[0] = DecToBcd(sec);	// 0x07
	Target[1] = DecToBcd(min);	// 0x08
	Target[2] = DecToBcd(hour);	// 0x09
	Target[3] = DecToBcd(dydt);	// 0x0A

	/* Mask 7 bit of register according to table */
	switch(mode)
	{
	case MODE_ALL:
		break;
	case MODE_HOUR_MIN_SEC:
		Target[3] |= (1 << 7);
		break;
	case MODE_MIN_SEC:
		Target[2] |= (1 << 7);
		Target[3] |= (1 << 7);
		break;
	case MODE_SEC:
		Target[1] |= (1 << 7);
		Target[2] |= (1 << 7);
		Target[3] |= (1 << 7);
		break;
	case MODE_ONCE_PER_SEC:
		Target[0] |= (1 << 7);
		Target[1] |= (1 << 7);
		Target[2] |= (1 << 7);
		Target[3] |= (1 << 7);
		break;
	default:
	    break;
	}

	/* Write Alarm 1 Registers */
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_ALARM_1, Target, 4);
	/* Enable Alarm 1 at Control Register */
	DS3231_EnableInterrupt(DS3231_Enabled);
	DS3231_Alarm1_Interrupt(DS3231_Enabled);
	/* Clear Alarm 1 status flag */
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	status &= ~(1 << DS3231_STAT_A1F);
	status |= (0 << DS3231_STAT_A1F);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
}

/**
  * @brief  Set Alarm 2 on RTC
  * @param  mode  	Matching mode, MODE_ALL, MODE_HOUR_MIN, MODE_MIN,
  * 				MODE_ONCE_PER_MIN
  * @param  min  	Minute, 0-60
  * @param  hour  	Hours, 0-23
  * @param  dydt 	Date or Day
  */
void DS3231_SetAlarm2(uint8_t mode, uint8_t min, uint8_t hour, uint8_t dydt)
{
	uint8_t status = 0x00;
	uint8_t Target[3];
	Target[0] = DecToBcd(min);	// 0x0B
	Target[1] = DecToBcd(hour);	// 0x0C
	Target[2] = DecToBcd(dydt);	// 0x0D

	/* Mask 7 bit of register according to table */
	switch(mode)
	{
	case MODE_ALL:
		if (ALARM_DYDT == 1)	Target[2] |= (1 << 6);
		break;
	case MODE_HOUR_MIN:
		Target[2] |= (1 << 7);
		break;
	case MODE_MIN:
		Target[1] |= (1 << 7);
		Target[2] |= (1 << 7);
		break;
	case MODE_ONCE_PER_MIN:
		Target[0] |= (1 << 7);
		Target[1] |= (1 << 7);
		Target[2] |= (1 << 7);
		break;
	default:
	    break;
	}

	/* Write Alarm 2 Registers */
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_ALARM_2, Target, 3);
	/* Enable Alarm 2 at Control Register */
	DS3231_EnableInterrupt(DS3231_Enabled);
	DS3231_Alarm2_Interrupt(DS3231_Enabled);
	/* Clear Alarm 2 status flag */
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	status &= ~(1 << DS3231_STAT_A2F);
	status |= (0 << DS3231_STAT_A2F);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
}

/**
  * @brief  Enable Convert Temperature
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_EnableConvTemp(uint8_t mode)
{
	uint8_t ctrl = 0x00;
	uint8_t status = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	// Waiting for BSY bit to clear
	while (DS3231_GetFlag(DS3231_STAT_BSY) == 1) {};
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(1 << DS3231_CTRL_CONV);
	ctrl |= (mode << DS3231_CTRL_CONV);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}

/**
  * @brief  Enable 32kHz Output
  * @param  mode  DS3231_Enabled | DS3231_Disable
  */
void DS3231_32kHzOutput(uint8_t mode)
{
	uint8_t status = 0x00;

	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
	status &= ~(1 << DS3231_STAT_EN32K);
	status |= (mode << DS3231_STAT_EN32K);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_STATUS, &status, 1);
}

/**
  * @brief  Set SQUARE-WAVE OUTPUT FREQUENCY
  * @param  freq  SQW_1HZ, SQW_1024HZ, SQW_4096HZ, SQW_8192HZ
  */
void DS3231_SetSQWFreq(SQWRate freq)
{
	uint8_t ctrl = 0x00;
	DS3231_Bus_Read(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
	ctrl &= ~(3 << DS3231_CTRL_RS1);
	ctrl |= (freq << DS3231_CTRL_RS1);
	DS3231_Bus_Write(DS3231_ADDRESS, DS3231_CONTROL, &ctrl, 1);
}
