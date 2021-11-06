#################################################################################################
#
#       stm32_sim800 : stm32f401 + sim800l(uart) + atgm332d(uart) + ssd1306(i2c)
#
#################################################################################################


## The composition of the working equipment:

```
* stm32f401 (new black pill board) - microcontroller board
* atgm332d - gps/glonass module (interface uart)
* ssd1306 - OLED display 0.96 "128x64 (interfaces I2C)
```


# Development Tools:

```
* STM32CubeMX - graphic package for creating projects (in C) for microcontrollers of the STM32 family
  (https://www.st.com/en/development-tools/stm32cubemx.html).
* System Workbench for STM32 - IDE software development environment for STM32 microcontrollers
  (https://www.st.com/en/development-tools/sw4stm32.html).
* stm32flash - utility for writing firmware to flash memory of STM32 microcontrollers
   via the built-in USART1 port (https://sourceforge.net/projects/stm32flash/files/)
* STM32CubeProgrammer - utility for writing firmware to flash memory of STM32 microcontrollers
  (https://www.st.com/en/development-tools/stm32cubeprog.html).
* ST-LINK V2 - usb debugger for STM8 / STM32 microcontrollers.
* Saleae USB Logic 8ch - logic signal analyzer, 8 channels, max. sampling rate 24MHz
  (https://www.saleae.com/ru/downloads/).
```

P.S.

This project is under development - testing project

