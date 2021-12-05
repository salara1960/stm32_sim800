################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ds18b20.c \
../Core/Src/fontik.c \
../Core/Src/freertos.c \
../Core/Src/func.c \
../Core/Src/gps.c \
../Core/Src/main.c \
../Core/Src/sms.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/w25.c 

OBJS += \
./Core/Src/ds18b20.o \
./Core/Src/fontik.o \
./Core/Src/freertos.o \
./Core/Src/func.o \
./Core/Src/gps.o \
./Core/Src/main.o \
./Core/Src/sms.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/w25.o 

C_DEPS += \
./Core/Src/ds18b20.d \
./Core/Src/fontik.d \
./Core/Src/freertos.d \
./Core/Src/func.d \
./Core/Src/gps.d \
./Core/Src/main.d \
./Core/Src/sms.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/w25.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -Og -ffunction-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ds18b20.d ./Core/Src/ds18b20.o ./Core/Src/fontik.d ./Core/Src/fontik.o ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/func.d ./Core/Src/func.o ./Core/Src/gps.d ./Core/Src/gps.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/sms.d ./Core/Src/sms.o ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/w25.d ./Core/Src/w25.o

.PHONY: clean-Core-2f-Src

