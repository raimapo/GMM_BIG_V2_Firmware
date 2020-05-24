/**
  ******************************************************************************
  * @file           : eeprom.h
  * @brief          : Header for eeprom.cpp file.
  *                   This file contains the common defines of the application.
  * @version		: 0.1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 Raimondas Pomarnacki.
  * All rights reserved.</center></h2>
  *
  * This software component is not licensed,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_24AA02UID_HPP
#define __EEPROM_24AA02UID_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "main.h"
#include "i2c.h"

#ifdef __cplusplus

#define MAX_WRITE_PAGE 8 //24AA02UID // 24AA025UID MAX_WRITE_PAGE 16

class EEPROM {

	I2C_HandleTypeDef* _i2c;

	public:

	/**
	 *	Constructor
	 */
	EEPROM(I2C_HandleTypeDef *hi2c);

	/**
	 * Initialiser
	 * Sets up the I2C interface
	 */
    void init(void);

    /**
     * Read data manually
     */
    uint8_t readData(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes);

    /**
     * Write data manually
     */
    void writeData(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes);

	private:

    // Variables
    uint8_t Address = 0x50;
    uint8_t position;
    uint32_t UID;
    uint8_t CompanyID;
    uint8_t EEPROMID;

    uint8_t readByte(uint8_t dataAddress);
    uint8_t readConsecutive(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes);
    void writeByte(uint8_t dataAddress, uint8_t data);
    void writePage(uint8_t * dataBuffer, uint8_t startAddress, uint8_t bytes);


    uint8_t readCompanyID(void);
    uint8_t readEEPROMID(void);
    uint32_t readUID(void);

};

#endif


#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_24AA02UID_H */
