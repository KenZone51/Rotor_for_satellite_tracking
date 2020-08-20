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
RotorControlRPI_CAN_Semihosting.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
RotorControlRPI_CAN_Semihosting.list \

OBJCOPY_BIN += \
RotorControlRPI_CAN_Semihosting.bin \


# All Target
all: RotorControlRPI_CAN_Semihosting.elf secondary-outputs

# Tool invocations
RotorControlRPI_CAN_Semihosting.elf: $(OBJS) $(USER_OBJS) /Users/kenza/Desktop/Embedded\ C/Workspace/Target/RotorControlRPI_CAN_Semihosting/STM32F072RBTX_FLASH.ld
	arm-none-eabi-gcc -o "RotorControlRPI_CAN_Semihosting.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/Users/kenza/Desktop/Embedded C/Workspace/Target/RotorControlRPI_CAN_Semihosting/STM32F072RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="RotorControlRPI_CAN_Semihosting.map" -Wl,--gc-sections -static -specs=rdimon.specs -lc -lrdimon --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

RotorControlRPI_CAN_Semihosting.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "RotorControlRPI_CAN_Semihosting.list"
	@echo 'Finished building: $@'
	@echo ' '

RotorControlRPI_CAN_Semihosting.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "RotorControlRPI_CAN_Semihosting.bin"
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
