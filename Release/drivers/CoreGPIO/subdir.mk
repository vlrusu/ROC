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
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\drivers\CoreGPIO" -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\hal" -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\hal\CortexM1" -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\CMSIS" -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\CMSIS\startup_gcc" -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\hal\CortexM1\GNU" -I"C:\Users\tecchio\Desktop\DRAC Software\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


