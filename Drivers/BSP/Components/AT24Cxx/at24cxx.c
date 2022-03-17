/**
  ******************************************************************************
  * @file    at24cxx.c
  * @brief   This file includes the HAL/LL driver for AT24C32/64 EEPROM
  ******************************************************************************
  */
#include "AT24Cxx.h"

/**
  * @brief  I2C Bus Write 16bit
  * @retval Success = 0, Failed = 1
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be Write
  */
uint8_t AT24Cxx_Bus_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData,
		uint16_t Len)
{
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
    LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT, 2,
    		I2C_CR2_RELOAD, LL_I2C_GENERATE_START_WRITE);

	 /* Send 16bit Memory Address */
    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)){};
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)((uint16_t)(((uint16_t)(MemAddr
    		& (uint16_t)(0xFF00U))) >> 8U))));	// MSB
    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)){};
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(MemAddr
    		& (uint16_t)0x00FF)));	// LSB

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
    if (HAL_I2C_Mem_Write(I2Cx, AT24Cxx_ADDRESS, MemAddr, I2C_MEMADD_SIZE_16BIT,
    		pData, AT24Cxx_PAGE_SIZE, HAL_MAX_DELAY) != HAL_OK)
    {
    	return 1;
    }
#endif

    return 0;
}

/**
  * @brief  I2C Bus Read 16bits
  * @retval Success = 0, Failed = 1
  * @param  DevAddr		Target device address
  * @param  MemAddr  	Internal memory address
  * @param  pData 		Pointer to data buffer
  * @param  Len  		Amount of data to be read
  */
uint8_t AT24Cxx_Bus_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t *pData,
		uint16_t Len)
{
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
    LL_I2C_HandleTransfer(I2Cx, DevAddr, LL_I2C_ADDRSLAVE_7BIT, 2,
    		LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

	 /* Send 16bit Memory Address */
    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)) {};
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)((uint16_t)(((uint16_t)(MemAddr
    		& (uint16_t)(0xFF00U))) >> 8U))));	// MSB
    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx)){};
    LL_I2C_TransmitData8(I2Cx, ((uint8_t)(uint16_t)(MemAddr
    		& (uint16_t)0x00FF)));	// LSB

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
	if (HAL_I2C_Mem_Read(I2Cx, AT24Cxx_ADDRESS, MemAddr, I2C_MEMADD_SIZE_16BIT,
			pData, Len, HAL_MAX_DELAY) != 0)
	{
		return 1;
	}
#endif

    return 0;
}

/**
  * @brief  Fill EEPROM page with selected value
  * @retval Success = 0, Failed = 1
  * @param  Page	Any page up to AT24Cxx_PAGE_NUM
  * @param  Value	Value to filled
  */
uint8_t AT24Cxx_FillPage(uint16_t Page, uint8_t Value)
{
	uint8_t Buffer[AT24Cxx_PAGE_SIZE];
	memset(Buffer, Value, AT24Cxx_PAGE_SIZE);
	uint16_t MemAddr = Page << (int)(log(AT24Cxx_PAGE_SIZE) / log(2));

	AT24Cxx_Bus_Write(AT24Cxx_ADDRESS, MemAddr, Buffer, AT24Cxx_PAGE_SIZE);

#ifdef LL_Driver
	LL_mDelay(5);	// 5ms Write cycle delay
#else
	HAL_Delay(5);	// 5ms Write cycle delay
#endif

	return 0;
}

/**
  * @brief  Erase EEPROM with value 0xFF
  * @retval Success = 0, Failed = 1
  */
uint8_t AT24Cxx_EraseChip(void)
{
	for (uint16_t i = 0; i < AT24Cxx_PAGE_NUM; i++)
	{
		if (AT24Cxx_FillPage(i, 0xFF) != 0)
		{
			return 1;
		}
	}

	return 0;
}

/**
  * @brief  Sequential read from selected memory address with number of byte
  * @retval Success = 0, Failed = 1
  * @param  MemAddr	Memory address to start read from
  * @param  pData	Pointer to data
  * @param  Len		Number of byte to read
  */
uint8_t AT24Cxx_ReadByte(uint16_t MemAddr, uint8_t *pData, uint16_t Len)
{
	AT24Cxx_Bus_Read(AT24Cxx_ADDRESS, MemAddr, pData, Len);

	return 0;
}

/**
  * @brief  Write data into EEPROM
  * @retval Success = 0, Failed = 1
  * @param  MemAddr	Memory address to start write from
  * @param  pData	Pointer to data
  * @param  Len		Number of byte to write
  */
uint8_t AT24Cxx_WriteByte(uint16_t MemAddr, uint8_t *value, uint16_t Len)
{
	while (Len > AT24Cxx_PAGE_SIZE)
	{
		uint16_t WrtSize = AT24Cxx_PAGE_SIZE - (MemAddr % AT24Cxx_PAGE_SIZE);

		AT24Cxx_Bus_Write(AT24Cxx_ADDRESS, MemAddr, value, WrtSize);
		value += WrtSize;
		Len -= WrtSize;
		MemAddr += WrtSize;
#ifdef LL_Driver
		LL_mDelay(5);	// 5ms Write cycle delay
#else
		HAL_Delay(5);	// 5ms Write cycle delay
#endif
	}
	/* Write the remaining data */
	AT24Cxx_Bus_Write(AT24Cxx_ADDRESS, MemAddr, value, Len);
#ifdef LL_Driver
	LL_mDelay(5);	// 5ms Write cycle delay
#else
	HAL_Delay(5);	// 5ms Write cycle delay
#endif

	return 0;
}
