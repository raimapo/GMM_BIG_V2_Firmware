################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
../Middlewares/Third_Party/FreeRTOS/Source/list.c \
../Middlewares/Third_Party/FreeRTOS/Source/queue.c \
../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
../Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
../Middlewares/Third_Party/FreeRTOS/Source/timers.c 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.d \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.d \
./Middlewares/Third_Party/FreeRTOS/Source/list.d \
./Middlewares/Third_Party/FreeRTOS/Source/queue.d \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.d \
./Middlewares/Third_Party/FreeRTOS/Source/timers.d 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/croutine.o \
./Middlewares/Third_Party/FreeRTOS/Source/event_groups.o \
./Middlewares/Third_Party/FreeRTOS/Source/list.o \
./Middlewares/Third_Party/FreeRTOS/Source/queue.o \
./Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o \
./Middlewares/Third_Party/FreeRTOS/Source/tasks.o \
./Middlewares/Third_Party/FreeRTOS/Source/timers.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/croutine.o: ../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/croutine.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/event_groups.o: ../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/event_groups.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/list.o: ../Middlewares/Third_Party/FreeRTOS/Source/list.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/list.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/queue.o: ../Middlewares/Third_Party/FreeRTOS/Source/queue.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/queue.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.o: ../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/tasks.o: ../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/tasks.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/Third_Party/FreeRTOS/Source/timers.o: ../Middlewares/Third_Party/FreeRTOS/Source/timers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 '-DUAVCAN_CPP_VERSION=UAVCAN_CPP11' -DUSE_HAL_DRIVER -DconfigUSE_NEWLIB_REENTRANT '-DUAVCAN_STM32_FREERTOS=1' -DSTM32F303xC -DARM_MATH_CM4 '-DUAVCAN_STM32_TIMER_NUMBER=2' '-D__FPU_PRESENT=1' -DDEBUG '-DUAVCAN_STM32_NUM_IFACES=1' -c -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Middlewares/libuavcan/libuavcan_drivers/stm32/driver/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Debug/dsdlc_generated -I../Drivers/CMSIS/DSP/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Core/Inc -I../Middlewares/libuavcan/libuavcan/include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/Third_Party/FreeRTOS/Source/timers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

