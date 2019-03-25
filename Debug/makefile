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
MyProj.hex \

SECONDARY_SIZE += \
MyProj.siz \


# All Target
all: MyProj.elf secondary-outputs

# Tool invocations
MyProj.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM Cross C Linker'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\blinky.ld" -Xlinker --gc-sections -Wl,-Map,"MyProj.map" -o "MyProj.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

MyProj.hex: MyProj.elf
	@echo 'Invoking: GNU ARM Cross Create Flash Image'
	arm-none-eabi-objcopy -O ihex "MyProj.elf"  "MyProj.hex"
	@echo 'Finished building: $@'
	@echo ' '

MyProj.siz: MyProj.elf
	@echo 'Invoking: GNU ARM Cross Print Size'
	arm-none-eabi-size --format=berkeley "MyProj.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) MyProj.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
