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
Start application version '1.8 (17.11.2021)'
17.11 15:35:51 [StartDefaultTask] Start main thread...(13416)
17.11 16:35:51 [StartTemp] Start sensor thread...(13416)
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
+CSQ: 15,0
OK
AT+CREG?
+CREG: 0,2
OK
AT+CREG?
+CREG: 0,1
OK
AT+CGATT=1
+CME ERROR: 100
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
+CCLK: "21/11/17,16:17:48+02"
Set date/time 17/11/21 16:17:48+02 OK !
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
17.10 16:20:11 time:142033 date:171121 lat:54.727718 lon:20.537056 sat:8 alt:9.8 spd:0.00 dir:181.19
17.10 16:20:11 time:142034 date:171121 lat:54.727718 lon:20.537054 sat:8 alt:9.8 spd:0.00 dir:181.19
17.10 16:20:12 time:142035 date:171121 lat:54.727718 lon:20.537054 sat:9 alt:9.8 spd:0.00 dir:181.19
17.10 16:20:13 time:142036 date:171121 lat:54.727718 lon:20.537054 sat:9 alt:9.8 spd:0.00 dir:181.19
17.10 16:20:14 time:142037 date:171121 lat:54.727718 lon:20.537054 sat:9 alt:9.8 spd:0.00 dir:181.19
17.10 16:20:15 time:142038 date:171121 lat:54.727718 lon:20.537054 sat:9 alt:10.0 spd:0.00 dir:181.19
17.10 16:20:16 time:142039 date:171121 lat:54.727718 lon:20.537052 sat:9 alt:10.0 spd:0.00 dir:181.19
17.10 16:20:17 time:142040 date:171121 lat:54.727718 lon:20.537052 sat:9 alt:10.1 spd:0.00 dir:181.19
.
.
.
offgps

radio
AT+FMOPEN=0
OK
AT+FMVOLUME=6
OK
AT+FMSCAN
1018
1025
OK
AT+FMFREQ=1025
OK
rlist
17.10 16:22:38 Radio freq_list MHz:[101.8 102.5]
AT+FMCLOSE
OK
```



P.S.

This project is under development and will be replenished.


