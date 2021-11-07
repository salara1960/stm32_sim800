################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fontik.c \
../Core/Src/freertos.c \
../Core/Src/func.c \
../Core/Src/gps.c \
../Core/Src/main.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/w25.c 

OBJS += \
./Core/Src/fontik.o \
./Core/Src/freertos.o \
./Core/Src/func.o \
./Core/Src/gps.o \
./Core/Src/main.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/w25.o 

C_DEPS += \
./Core/Src/fontik.d \
./Core/Src/freertos.d \
./Core/Src/func.d \
./Core/Src/gps.d \
./Core/Src/main.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/w25.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DUSE_HAL_DRIVER -DSTM32F401xC -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Core/Inc" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/alarm/Project/STM32/TEST/stm32_bridge/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


