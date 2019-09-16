################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include hal/CortexM1/GNU/subdir.mk
-include hal/CortexM1/subdir.mk
-include drivers/CoreUARTapb/subdir.mk
-include drivers/CoreSysServices_PF/subdir.mk
-include drivers/CoreSPI/subdir.mk
-include drivers/CorePWM/subdir.mk
-include drivers/CoreGPIO/subdir.mk
-include CMSIS/startup_gcc/subdir.mk
-include CMSIS/subdir.mk
-include .metadata/.plugins/org.eclipse.cdt.make.core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
ROC_softconsole.hex \

SECONDARY_SIZE += \
ROC_softconsole.siz \


# All Target
all: ROC_softconsole.elf secondary-outputs

# Tool invocations
ROC_softconsole.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM Cross C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -T "C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\blinky.ld" -Xlinker --gc-sections -Wl,-Map,"ROC_softconsole.map" -Xlinker --print-map --specs=nano.specs -v -o "ROC_softconsole.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

ROC_softconsole.hex: ROC_softconsole.elf
	@echo 'Invoking: GNU ARM Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "ROC_softconsole.elf"  "ROC_softconsole.hex"
	@echo 'Finished building: $@'
	@echo ' '

ROC_softconsole.siz: ROC_softconsole.elf
	@echo 'Invoking: GNU ARM Cross Print Size'
	arm-none-eabi-size --format=berkeley --totals "ROC_softconsole.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) ROC_softconsole.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
