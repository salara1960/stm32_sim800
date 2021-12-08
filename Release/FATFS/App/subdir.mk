################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FATFS/App/fatfs.c 

OBJS += \
./FATFS/App/fatfs.o 

C_DEPS += \
./FATFS/App/fatfs.d 


# Each subdirectory must supply rules for building sources it contributes
FATFS/App/%.o: ../FATFS/App/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F401xC -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Core/Inc" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/CMSIS/Include" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/FATFS/Target" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/FATFS/App" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FatFs/src"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


