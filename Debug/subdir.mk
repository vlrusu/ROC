################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AD5318.c \
../Commands.c \
../DS28CM00.c \
../HDC2080.c \
../I2C.c \
../LTC2634.c \
../MCP23S17.c \
../MCP3427.c \
../autobitslip.c \
../bme280.c \
../main.c \
../utils.c 

OBJS += \
./AD5318.o \
./Commands.o \
./DS28CM00.o \
./HDC2080.o \
./I2C.o \
./LTC2634.o \
./MCP23S17.o \
./MCP3427.o \
./autobitslip.o \
./bme280.o \
./main.o \
./utils.o 

C_DEPS += \
./AD5318.d \
./Commands.d \
./DS28CM00.d \
./HDC2080.d \
./I2C.d \
./LTC2634.d \
./MCP23S17.d \
./MCP3427.d \
./autobitslip.d \
./bme280.d \
./main.d \
./utils.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\vadim\OneDrive\Documents\ROC\drivers\CoreGPIO" -I"C:\Users\vadim\OneDrive\Documents\ROC\hal" -I"C:\Users\vadim\OneDrive\Documents\ROC\hal\CortexM1" -I"C:\Users\vadim\OneDrive\Documents\ROC\CMSIS" -I"C:\Users\vadim\OneDrive\Documents\ROC\CMSIS\startup_gcc" -I"C:\Users\vadim\OneDrive\Documents\ROC\hal\CortexM1\GNU" -I"C:\Users\vadim\OneDrive\Documents\ROC\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


