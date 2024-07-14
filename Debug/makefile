################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Utilities/trace/adv_trace/subdir.mk
-include Utilities/timer/subdir.mk
-include Utilities/sequencer/subdir.mk
-include Utilities/misc/subdir.mk
-include Utilities/lpm/tiny_lpm/subdir.mk
-include Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Utilities/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Mac/Region/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Mac/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/LmHandler/Packages/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/LmHandler/subdir.mk
-include Middlewares/Third_Party/LoRaWAN/Crypto/subdir.mk
-include LoRaWAN/Target/subdir.mk
-include LoRaWAN/App/subdir.mk
-include Drivers/STM32WLxx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
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


BUILD_ARTIFACT_NAME := pulsewan_code
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
pulsewan_code.elf \

MAP_FILES += \
pulsewan_code.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
pulsewan_code.list \


# All Target
all: main-build

# Main-build Target
main-build: pulsewan_code.elf secondary-outputs

# Tool invocations
pulsewan_code.elf pulsewan_code.map: $(OBJS) $(USER_OBJS) /home/mauricio/STM32CubeIDE/workspace_1.16.0/pulsewan_code/STM32WLE5CCUX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "pulsewan_code.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/mauricio/STM32CubeIDE/workspace_1.16.0/pulsewan_code/STM32WLE5CCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="pulsewan_code.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

pulsewan_code.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "pulsewan_code.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout pulsewan_code.elf pulsewan_code.list pulsewan_code.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
