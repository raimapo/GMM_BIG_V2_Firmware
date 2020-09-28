################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Middlewares/libuavcan/libuavcan/src/driver/uc_can.cpp 

OBJS += \
./Middlewares/libuavcan/libuavcan/src/driver/uc_can.o 

CPP_DEPS += \
./Middlewares/libuavcan/libuavcan/src/driver/uc_can.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/libuavcan/libuavcan/src/driver/uc_can.o: ../Middlewares/libuavcan/libuavcan/src/driver/uc_can.cpp
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"Middlewares/libuavcan/libuavcan/src/driver/uc_can.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

