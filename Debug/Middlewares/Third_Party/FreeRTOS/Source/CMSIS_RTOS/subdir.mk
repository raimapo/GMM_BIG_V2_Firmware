################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

