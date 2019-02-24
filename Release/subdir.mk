################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AD5318.c \
../Commands.c \
../DS28CM00.c \
../I2C.c \
../LTC2634.c \
../MCP23S17.c \
../MCP3427.c \
../bme280.c \
../main.c \
../utils.c 

OBJS += \
./AD5318.o \
./Commands.o \
./DS28CM00.o \
./I2C.o \
./LTC2634.o \
./MCP23S17.o \
./MCP3427.o \
./bme280.o \
./main.o \
./utils.o 

C_DEPS += \
./AD5318.d \
./Commands.d \
./DS28CM00.d \
./I2C.d \
./LTC2634.d \
./MCP23S17.d \
./MCP3427.d \
./bme280.d \
./main.d \
./utils.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\drivers\CoreGPIO" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\hal" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\hal\CortexM1" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\CMSIS" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\CMSIS\startup_gcc" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\hal\CortexM1\GNU" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


