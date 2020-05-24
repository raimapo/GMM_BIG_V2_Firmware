#include "MicroSeconds.h"

/**
 * Initialization routine.
 * You might need to enable access to DWT registers on Cortex-M7
 *   DWT->LAR = 0xC5ACCE55
 */
void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        //DWT->LAR = 0xC5ACCE55
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

/**
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * No need to check an overflow. Let it just tick :)
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
void DWT_Delay(uint32_t us) // microseconds
{
    uint32_t startTick = DWT->CYCCNT,
             delayTicks = us * (SystemCoreClock/1000000);

    while (DWT->CYCCNT - startTick < delayTicks);
}



/*
 * Calculating systics for varios timers
 */

uint32_t micros(void)
{
	uint32_t us_ticks = 0;
	uint32_t uptime_ticks = 0;
	uint32_t cycle_ticks = 0;

	register uint32_t old_cycle, cycle, timeMs;
	us_ticks = SystemCoreClock / 1000000u;

	do{
		timeMs = __LDREXW(&uptime_ticks);
		cycle = DWT->CYCCNT;
		old_cycle = cycle_ticks;
	}
	while ( __STREXW( timeMs , &uptime_ticks ) );
	return (timeMs * 1000.0) + (cycle - old_cycle) / us_ticks;
}

uint32_t milis(void)
{
	//return HAL_GetTick();
	return micros() / 1000.0;
}
