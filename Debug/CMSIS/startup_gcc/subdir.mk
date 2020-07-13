################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/startup_gcc/newlib_stubs.c \
../CMSIS/startup_gcc/sys_boot.c 

S_UPPER_SRCS += \
../CMSIS/startup_gcc/startup_cortexm1_cfg.S 

OBJS += \
./CMSIS/startup_gcc/newlib_stubs.o \
./CMSIS/startup_gcc/startup_cortexm1_cfg.o \
./CMSIS/startup_gcc/sys_boot.o 

S_UPPER_DEPS += \
./CMSIS/startup_gcc/startup_cortexm1_cfg.d 

C_DEPS += \
./CMSIS/startup_gcc/newlib_stubs.d \
./CMSIS/startup_gcc/sys_boot.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/startup_gcc/%.o: ../CMSIS/startup_gcc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -I"C:\Users\vrusu\workspace\ROC\drivers\CoreGPIO" -I"C:\Users\vrusu\workspace\ROC\hal" -I"C:\Users\vrusu\workspace\ROC\hal\CortexM1" -I"C:\Users\vrusu\workspace\ROC\CMSIS" -I"C:\Users\vrusu\workspace\ROC\CMSIS\startup_gcc" -I"C:\Users\vrusu\workspace\ROC\hal\CortexM1\GNU" -I"C:\Users\vrusu\workspace\ROC\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

CMSIS/startup_gcc/%.o: ../CMSIS/startup_gcc/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


