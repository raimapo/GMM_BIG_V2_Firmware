/**
  ******************************************************************************
  * @file           : MicroSeconds.h
  * @brief          : Header for MicroSeconds.c file.
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

#ifndef __MICROSECONDS_H
#define __MICROSECONDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * Initialization routine.
 * You might need to enable access to DWT registers on Cortex-M7
 *   DWT->LAR = 0xC5ACCE55
 */
void DWT_Init(void);

/**
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * No need to check an overflow. Let it just tick :)
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
void DWT_Delay(uint32_t us);

/*
 * Calculating systics for varios timers
 */

uint32_t micros(void);
uint32_t milis(void);

#ifdef __cplusplus
}
#endif

#endif /* __MICROSECONDS_H */
