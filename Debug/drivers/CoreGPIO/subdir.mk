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
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\drivers\CoreGPIO" -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\hal" -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\hal\CortexM1" -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\CMSIS" -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\CMSIS\startup_gcc" -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\hal\CortexM1\GNU" -I"C:\Users\vrusu\Desktop\DRAC2FW\FW-01-2019\ROC_1-8-19\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


