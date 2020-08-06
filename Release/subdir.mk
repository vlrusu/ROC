################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../AD5318.c \
../ADC124S051.c \
../Commands.c \
../DS28CM00.c \
../HDC2080.c \
../I2C.c \
../LTC2634.c \
../MCP23S17.c \
../MCP3427.c \
../SPI_daisy.c \
../autobitslip.c \
../bme280.c \
../bme280_SPI_daisy.c \
../main.c \
../utils.c 

OBJS += \
./AD5318.o \
./ADC124S051.o \
./Commands.o \
./DS28CM00.o \
./HDC2080.o \
./I2C.o \
./LTC2634.o \
./MCP23S17.o \
./MCP3427.o \
./SPI_daisy.o \
./autobitslip.o \
./bme280.o \
./bme280_SPI_daisy.o \
./main.o \
./utils.o 

C_DEPS += \
./AD5318.d \
./ADC124S051.d \
./Commands.d \
./DS28CM00.d \
./HDC2080.d \
./I2C.d \
./LTC2634.d \
./MCP23S17.d \
./MCP3427.d \
./SPI_daisy.d \
./autobitslip.d \
./bme280.d \
./bme280_SPI_daisy.d \
./main.d \
./utils.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\drivers\CoreGPIO" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\hal" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\hal\CortexM1" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\CMSIS" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\CMSIS\startup_gcc" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\hal\CortexM1\GNU" -I"E:\UM-research\SoftConsole\ROCDIGI_02_25_19\SoftConsole\MyProj\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


