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
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\vadim\OneDrive\Documents\ROC\drivers\CoreGPIO" -I"C:\Users\vadim\OneDrive\Documents\ROC\hal" -I"C:\Users\vadim\OneDrive\Documents\ROC\hal\CortexM1" -I"C:\Users\vadim\OneDrive\Documents\ROC\CMSIS" -I"C:\Users\vadim\OneDrive\Documents\ROC\CMSIS\startup_gcc" -I"C:\Users\vadim\OneDrive\Documents\ROC\hal\CortexM1\GNU" -I"C:\Users\vadim\OneDrive\Documents\ROC\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


