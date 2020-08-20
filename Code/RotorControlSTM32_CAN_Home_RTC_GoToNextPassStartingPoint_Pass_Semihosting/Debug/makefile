################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.list \

OBJCOPY_BIN += \
RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.bin \


# All Target
all: RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.elf secondary-outputs

# Tool invocations
RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.elf: $(OBJS) $(USER_OBJS) C:\Users\Kenza\Desktop\Travail\Embedded\ C\Workspace\Target\RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting\STM32F072RBTX_FLASH.ld
	arm-none-eabi-gcc -o "RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\Kenza\Desktop\Travail\Embedded C\Workspace\Target\RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting\STM32F072RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.map" -Wl,--gc-sections -static -specs=rdimon.specs -lc -lrdimon --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.list"
	@echo 'Finished building: $@'
	@echo ' '

RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RotorControlSTM32_CAN_Home_RTC_GoToNextPassStartingPoint_Pass_Semihosting.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
