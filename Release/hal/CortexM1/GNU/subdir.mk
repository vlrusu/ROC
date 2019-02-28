################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../hal/CortexM1/GNU/hal.S \
../hal/CortexM1/GNU/hw_reg_access.S 

OBJS += \
./hal/CortexM1/GNU/hal.o \
./hal/CortexM1/GNU/hw_reg_access.o 

S_UPPER_DEPS += \
./hal/CortexM1/GNU/hal.d \
./hal/CortexM1/GNU/hw_reg_access.d 


# Each subdirectory must supply rules for building sources it contributes
hal/CortexM1/GNU/%.o: ../hal/CortexM1/GNU/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


