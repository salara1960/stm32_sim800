##############################################################
#
# stm32_sim800 : stm32f401+sim800l+atgm332d+ssd1306+ds18b20
#
##############################################################


## The composition of the working equipment:

```
* STM32F401 (new black pill board) - microcontroller board
* SIM800l - gsp/gprs module (interface uart)
* ATGM332D - gps/glonass module (interface uart)
* SSD1306 - OLED display 0.96 "128x64 (interfaces I2C)
* W25Q64 - flash memory chip with a capacity of 8MB (interfaces spi)
* DS18B20 - temperature sensor (interfaces 1-ware)
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

# Functional (will be replenished):

* The device uses freeRTOS software:
  - StartDefaultTask - task that support AT-commands nodule SIM800l.
  - StartGps - task that works with GPS data (NMEA) of the ATGM332D module.
* The device initializes some microcontroller interfaces:
  - GPIO : three LEDs are used: PB0,PB9,PB10, one pin (PA0 with button) reset sim800l, pin PB12 - for working sensor DS18B20
  - I2C1 : master mode with a frequency of 400KHz (the bus serves ssd1306).
  - USART1 : parameters of port 115200 8N1 (transmit data with DMA) - a port for logging and receive any commands for control device.
  - USART2 : parameters of port 115200 8N1 (transmit data with DMA)- a port for send AT command to SIM800l module.
  - USART6 : parameters of port 9600 8N1 - the port for receive NMEA data from GPS module ATGM332D.
  - TIM3 : Timer 10 ms. and 1 second, implemented in the callback function.
  - TIM10 : Timer 1 us., for working temperature sensor DS18B20.
  - RTC : real time clock, can be set using the command 'epoch_time=value:time_zone' or via sntp service
  - SPI1 : serves data-flash w25q64 : CS(PA4), SCK(PA5), MISO(PA6), MOSI(PA7) - this chip not used yet.
* The system timer (TIM1) counts milliseconds from the start of operation of the device.
* Data reception on all serial ports (USART1, USART2, USART6) is performed in the callback function of the interrupt handler.
  Received data is transferred to the corresponding tasks through specific queues.
* Support receive sms messages (with concat parts of messages).
* Support FM radio onchip SIM800l.


AT-commands example:

```
Start application version '1.6 (15.11.2021)'
15.11 22:06:43 Start main thread...(15456)
15.11 23:06:43 Start sensor thread...(15456)
RDY
+CFUN: 1
+CPIN: READY
Call Ready
SMS Ready
AT
OK
ATE0
OK
AT+CMEE=1
OK
AT+CLTS=1
OK
AT+CMGF=0
OK
AT+CNMI=1,2,0,1,0
OK
AT+GMR
Revision:1418B04SIM800L24
OK
AT+GSN
864369032292264
OK
AT+CSQ
+CSQ: 0,0
OK
AT+CSQ
+CSQ: 13,0
OK
AT+CREG?
+CREG: 0,1
OK
AT+CGATT=1
OK
AT+CIPSHUT
SHUT OK
AT+CIPMODE=0
OK
AT+CIPMUX=0
OK
AT+SAPBR=3,1,"CONTYPE","GPRS"
OK
AT+SAPBR=3,1,"APN","internet"
OK
AT+SAPBR=1,1
OK
AT+CNTPCID=1
OK
AT+CNTP="pool.ntp.org",8
OK
AT+CNTP
OK
+CNTP: 1
AT+CCLK?
+CCLK: "21/11/15,22:09:11+02"
Set date/time 15/11/21 22:09:11+02 OK !
OK
AT+SAPBR=0,1
OK
at+cusd=1,"*102#"
OK
+CUSD: 0, "003200300030002E0030003000200440002E", 72
200.00 р.
```


Console commands:

```
ongps
15.10 22:09:30 time:200934 date:151121 lat:54.728221 lon:20.536935 sat:5 alt:83.6 spd:0.00 dir:179.80
15.10 22:09:31 time:200935 date:151121 lat:54.728221 lon:20.536933 sat:5 alt:83.9 spd:0.00 dir:179.80
15.10 22:09:32 time:200936 date:151121 lat:54.728221 lon:20.536932 sat:5 alt:84.0 spd:0.00 dir:179.80
15.10 22:09:32 time:200937 date:151121 lat:54.728221 lon:20.536930 sat:5 alt:84.4 spd:0.00 dir:179.80
15.10 22:09:33 time:200938 date:151121 lat:54.728221 lon:20.536928 sat:5 alt:84.6 spd:0.00 dir:179.80
15.10 22:09:34 time:200939 date:151121 lat:54.728221 lon:20.536926 sat:5 alt:84.9 spd:0.00 dir:179.80
15.10 22:09:35 time:200940 date:151121 lat:54.728221 lon:20.536924 sat:5 alt:85.1 spd:0.00 dir:179.80
15.10 22:09:36 time:200941 date:151121 lat:54.728221 lon:20.536922 sat:5 alt:85.4 spd:0.00 dir:179.80
15.10 22:09:37 time:200942 date:151121 lat:54.728221 lon:20.536922 sat:5 alt:86.0 spd:0.00 dir:179.80
.
.
.
offgps
```



P.S.

This project is under development and will be replenished.


