/**
  ******************************************************************************
  * @file           : ina226.hpp
  * @brief          : Header for ina226.cpp file.
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

#ifndef __INA226_HPP
#define __INA226_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "main.h"
#include "math.h"
#include "i2c.h"

#define INA226_ADDRESS              (0x40)

#define INA226_REG_CONFIG           (0x00)
#define INA226_REG_SHUNTVOLTAGE     (0x01)
#define INA226_REG_BUSVOLTAGE       (0x02)
#define INA226_REG_POWER            (0x03)
#define INA226_REG_CURRENT          (0x04)
#define INA226_REG_CALIBRATION      (0x05)
#define INA226_REG_MASKENABLE       (0x06)
#define INA226_REG_ALERTLIMIT       (0x07)

#define INA226_BIT_SOL              (0x8000)
#define INA226_BIT_SUL              (0x4000)
#define INA226_BIT_BOL              (0x2000)
#define INA226_BIT_BUL              (0x1000)
#define INA226_BIT_POL              (0x0800)
#define INA226_BIT_CNVR             (0x0400)
#define INA226_BIT_AFF              (0x0010)
#define INA226_BIT_CVRF             (0x0008)
#define INA226_BIT_OVF              (0x0004)
#define INA226_BIT_APOL             (0x0002)
#define INA226_BIT_LEN              (0x0001)

typedef enum
{
    INA226_AVERAGES_1             = 0b000,
    INA226_AVERAGES_4             = 0b001,
    INA226_AVERAGES_16            = 0b010,
    INA226_AVERAGES_64            = 0b011,
    INA226_AVERAGES_128           = 0b100,
    INA226_AVERAGES_256           = 0b101,
    INA226_AVERAGES_512           = 0b110,
    INA226_AVERAGES_1024          = 0b111
} ina226_averages_t;

typedef enum
{
    INA226_BUS_CONV_TIME_140US    = 0b000,
    INA226_BUS_CONV_TIME_204US    = 0b001,
    INA226_BUS_CONV_TIME_332US    = 0b010,
    INA226_BUS_CONV_TIME_588US    = 0b011,
    INA226_BUS_CONV_TIME_1100US   = 0b100,
    INA226_BUS_CONV_TIME_2116US   = 0b101,
    INA226_BUS_CONV_TIME_4156US   = 0b110,
    INA226_BUS_CONV_TIME_8244US   = 0b111
} ina226_busConvTime_t;

typedef enum
{
    INA226_SHUNT_CONV_TIME_140US   = 0b000,
    INA226_SHUNT_CONV_TIME_204US   = 0b001,
    INA226_SHUNT_CONV_TIME_332US   = 0b010,
    INA226_SHUNT_CONV_TIME_588US   = 0b011,
    INA226_SHUNT_CONV_TIME_1100US  = 0b100,
    INA226_SHUNT_CONV_TIME_2116US  = 0b101,
    INA226_SHUNT_CONV_TIME_4156US  = 0b110,
    INA226_SHUNT_CONV_TIME_8244US  = 0b111
} ina226_shuntConvTime_t;

typedef enum
{
    INA226_MODE_POWER_DOWN      = 0b000,
    INA226_MODE_SHUNT_TRIG      = 0b001,
    INA226_MODE_BUS_TRIG        = 0b010,
    INA226_MODE_SHUNT_BUS_TRIG  = 0b011,
    INA226_MODE_ADC_OFF         = 0b100,
    INA226_MODE_SHUNT_CONT      = 0b101,
    INA226_MODE_BUS_CONT        = 0b110,
    INA226_MODE_SHUNT_BUS_CONT  = 0b111,
} ina226_mode_t;


#ifdef __cplusplus

class INA226
{
	I2C_HandleTypeDef* _i2c;

    public:

	INA226(I2C_HandleTypeDef *hi2c);

	bool init(uint8_t Address = INA226_ADDRESS);
	bool configure(ina226_averages_t avg = INA226_AVERAGES_1, ina226_busConvTime_t busConvTime = INA226_BUS_CONV_TIME_1100US, ina226_shuntConvTime_t shuntConvTime = INA226_SHUNT_CONV_TIME_1100US, ina226_mode_t mode = INA226_MODE_SHUNT_BUS_CONT);
	bool calibrate(float rShuntValue = 0.1, float iMaxExcepted = 2);
	void checkConfig(void);

	ina226_averages_t getAverages(void);
	ina226_busConvTime_t getBusConversionTime(void);
	ina226_shuntConvTime_t getShuntConversionTime(void);
	ina226_mode_t getMode(void);

	void enableShuntOverLimitAlert(void);
	void enableShuntUnderLimitAlert(void);
	void enableBusOvertLimitAlert(void);
	void enableBusUnderLimitAlert(void);
	void enableOverPowerLimitAlert(void);
	void enableConversionReadyAlert(void);

	void setBusVoltageLimit(float voltage);
	void setShuntVoltageLimit(float voltage);
	void setPowerLimit(float watts);

	void setAlertInvertedPolarity(bool inverted);
	void setAlertLatch(bool latch);

	bool isMathOverflow(void);
	bool isAlert(void);

	float readShuntCurrent(void);
	float readShuntVoltage(void);
	float readBusPower(void);
	float readBusVoltage(void);

	float getMaxPossibleCurrent(void);
	float getMaxCurrent(void);
	float getMaxShuntVoltage(void);
	float getMaxPower(void);

    private:

	int8_t inaAddress;
	float currentLSB, powerLSB;
	float vShuntMax, vBusMax, rShunt;

	void setMaskEnable(uint16_t mask);
	uint16_t getMaskEnable(void);

	void writeRegister16(uint8_t reg, uint16_t val);
	int16_t readRegister16(uint8_t reg);
};

#endif

#ifdef __cplusplus
}
#endif

#endif /* __INA226_HPP */
