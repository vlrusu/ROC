################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/CoreSPI/core_spi.c 

OBJS += \
./drivers/CoreSPI/core_spi.o 

C_DEPS += \
./drivers/CoreSPI/core_spi.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/CoreSPI/%.o: ../drivers/CoreSPI/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\drivers\CoreGPIO" -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\hal" -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\hal\CortexM1" -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\CMSIS" -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\CMSIS\startup_gcc" -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\hal\CortexM1\GNU" -I"E:\UM-research\SoftConsole\ROC_02-18-19\SoftConsole\myproj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


