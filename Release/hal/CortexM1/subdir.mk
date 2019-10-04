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
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\drivers\CoreGPIO" -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\hal" -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\hal\CortexM1" -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\CMSIS" -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\CMSIS\startup_gcc" -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\hal\CortexM1\GNU" -I"C:\Users\tecchio\Desktop\DRAC STP files\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


