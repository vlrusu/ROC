################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/CoreGPIO/core_gpio.c 

OBJS += \
./drivers/CoreGPIO/core_gpio.o 

C_DEPS += \
./drivers/CoreGPIO/core_gpio.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/CoreGPIO/%.o: ../drivers/CoreGPIO/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\drivers\CoreGPIO" -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\hal" -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\hal\CortexM1" -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\CMSIS" -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\CMSIS\startup_gcc" -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\hal\CortexM1\GNU" -I"C:\mu2e\polarfire\ROC_softconsole2\ROC_softconsole\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


