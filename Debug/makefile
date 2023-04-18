################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/nrf24L01/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := nrf_TX_RX
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
nrf_TX_RX.elf \

MAP_FILES += \
nrf_TX_RX.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
nrf_TX_RX.list \


# All Target
all: main-build

# Main-build Target
main-build: nrf_TX_RX.elf secondary-outputs

# Tool invocations
nrf_TX_RX.elf nrf_TX_RX.map: $(OBJS) $(USER_OBJS) /home/odemki/my_files/my_files/my_projects/STM32/FreeRTOS/15.Robo_hand_v2.0/1.nrf_RX_TX/STM32F401CCUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "nrf_TX_RX.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/odemki/my_files/my_files/my_projects/STM32/FreeRTOS/15.Robo_hand_v2.0/1.nrf_RX_TX/STM32F401CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="nrf_TX_RX.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

nrf_TX_RX.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "nrf_TX_RX.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout nrf_TX_RX.elf nrf_TX_RX.list nrf_TX_RX.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
