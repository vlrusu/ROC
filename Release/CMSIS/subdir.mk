################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/system_cortexm1_cfg.c 

OBJS += \
./CMSIS/system_cortexm1_cfg.o 

C_DEPS += \
./CMSIS/system_cortexm1_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/%.o: ../CMSIS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m1 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin  -g -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\drivers\CoreGPIO" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\hal" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\hal\CortexM1" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\CMSIS" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\CMSIS\startup_gcc" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\hal\CortexM1\GNU" -I"C:\Users\vrusu\Documents\GitHub\ARMCode\ROC\drivers\CoreUARTapb" -std=gnu11 --specs=cmsis.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


