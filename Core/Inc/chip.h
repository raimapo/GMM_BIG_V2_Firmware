#ifndef CHIP_UAVCAN_CONFIG_H
#define CHIP_UAVCAN_CONFIG_H

#include <stm32f3xx.h>

#define STM32F3XX

#define CAN1_TX_IRQHandler CAN_TX_IRQHandler
#define CAN1_RX0_IRQHandler CAN_RX0_IRQHandler
#define CAN1_RX1_IRQHandler CAN_RX1_IRQHandler

#define STM32_PCLK1 (36000000ul)
#define STM32_TIMCLK1 (72000000ul)

#endif
