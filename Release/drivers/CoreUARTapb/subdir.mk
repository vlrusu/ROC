################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/CoreUARTapb/core_uart_apb.c 

OBJS += \
./drivers/CoreUARTapb/core_uart_apb.o 

C_DEPS += \
./drivers/CoreUARTapb/core_uart_apb.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/CoreUARTapb/%.o: ../drivers/CoreUARTapb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\drivers\CoreGPIO" -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\hal" -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\hal\CortexM1" -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\CMSIS" -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\CMSIS\startup_gcc" -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\hal\CortexM1\GNU" -I"C:\Users\vrusu\Desktop\DRAC2FW\ROC\ROC_12-20-18\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


