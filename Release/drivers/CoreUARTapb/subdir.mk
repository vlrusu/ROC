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
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\drivers\CoreGPIO" -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\hal" -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\hal\CortexM1" -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\CMSIS" -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\CMSIS\startup_gcc" -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\hal\CortexM1\GNU" -I"D:\ywu\SoftConsole_200518\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


