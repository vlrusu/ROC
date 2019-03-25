################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hal/CortexM1/cortex_nvic.c 

OBJS += \
./hal/CortexM1/cortex_nvic.o 

C_DEPS += \
./hal/CortexM1/cortex_nvic.d 


# Each subdirectory must supply rules for building sources it contributes
hal/CortexM1/%.o: ../hal/CortexM1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\drivers\CoreGPIO" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\hal" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\hal\CortexM1" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\CMSIS" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\CMSIS\startup_gcc" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\hal\CortexM1\GNU" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19_SCv6\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


