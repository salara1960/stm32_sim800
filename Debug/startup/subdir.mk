################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f401xc.s 

OBJS += \
./startup/startup_stm32f401xc.o 

S_DEPS += \
./startup/startup_stm32f401xc.d 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-startup

clean-startup:
	-$(RM) ./startup/startup_stm32f401xc.d ./startup/startup_stm32f401xc.o

.PHONY: clean-startup

