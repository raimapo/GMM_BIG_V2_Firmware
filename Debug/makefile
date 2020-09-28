################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/libuavcan/libuavcan_drivers/stm32/driver/src/subdir.mk
-include Middlewares/libuavcan/libuavcan/src/transport/subdir.mk
-include Middlewares/libuavcan/libuavcan/src/protocol/subdir.mk
-include Middlewares/libuavcan/libuavcan/src/node/subdir.mk
-include Middlewares/libuavcan/libuavcan/src/marshal/subdir.mk
-include Middlewares/libuavcan/libuavcan/src/driver/subdir.mk
-include Middlewares/libuavcan/libuavcan/src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Drivers/CMSIS/DSP/Source/TransformFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/SupportFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/StatisticsFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/MatrixFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/FilteringFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/FastMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/ControllerFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/ComplexMathFunctions/subdir.mk
-include Drivers/CMSIS/DSP/Source/CommonTables/subdir.mk
-include Drivers/CMSIS/DSP/Source/BasicMathFunctions/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
GMM_BIG_V2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
GMM_BIG_V2.list \

OBJCOPY_BIN += \
GMM_BIG_V2.bin \


# All Target
all:
	+@make --no-print-directory pre-build && make --no-print-directory main-build

# Main-build Target
main-build: GMM_BIG_V2.elf secondary-outputs

# Tool invocations
GMM_BIG_V2.elf: $(OBJS) $(USER_OBJS) C:\Users\Raimondas\Desktop\GMM_BIG_V2_Firmware\STM32F303CCTX_FLASH.ld
	arm-none-eabi-g++ -o "GMM_BIG_V2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Raimondas\Desktop\GMM_BIG_V2_Firmware\STM32F303CCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="GMM_BIG_V2.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

GMM_BIG_V2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "GMM_BIG_V2.list"
	@echo 'Finished building: $@'
	@echo ' '

GMM_BIG_V2.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "GMM_BIG_V2.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

pre-build:
	python ../Middlewares/libuavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc ../Middlewares/libuavcan/dsdl/uavcan ../Middlewares/msg
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY: main-build pre-build

-include ../makefile.targets
