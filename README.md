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
* Send data (from sensor ds18b20 and gps module atgm332d) to external tcp_server in json format (every 10 sec.)


AT-commands example:

```
Start application version '1.9.1 (21.11.2021)'
21.11 15:16:45 [StartDefaultTask] Start main thread...(14952)
21.11 15:16:45 [StartTemp] Start sensor thread...(14952)
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
AT+CBC
+CBC: 0,74,3994
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
+CSQ: 13,0
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
+CCLK: "21/11/21,20:09:59+02"
Set date/time 21/11/21 20:09:59+02 OK !
OK
AT+SAPBR=0,1
OK
AT+CIPSTATUS
OK
STATE: IP INITIAL
AT+CSTT="internet","beeline","beeline"
OK
AT+CIPSTATUS
OK
STATE: IP START
AT+CIICR
OK
AT+CIPSTATUS
OK
STATE: IP GPRSACT
AT+CIFSR
10.215.109.252
AT+CIPSTATUS
OK
STATE: IP STATUS
AT+CIPSTART="TCP","10.10.10.10",8778
OK
CONNECT OK
AT+CIPSEND
>{"dev":"STM32_SIM800l","temp":27.0,"lat":54.7279,"lon":20.5368,"sat":7,"alt":71,"spd":0.00,"dir":24.65}
 
SEND OK
AT+CIPSEND
>{"dev":"STM32_SIM800l","temp":27.0,"lat":54.7279,"lon":20.5368,"sat":7,"alt":71,"spd":0.00,"dir":24.65}
 
SEND OK
.
.
.
AT+CIPSEND
>{"dev":"STM32_SIM800l","temp":27.6,"lat":54.7279,"lon":20.5367,"sat":8,"alt":76,"spd":0.00,"dir":1.28}
 
SEND OK
at+cipclose
CLOSE OK
AT+CIPSHUT
SHUT OK

at+cusd=1,"*102#"
OK
+CUSD: 0, "003200300030002E0030003000200440002E", 72
200.00 р.
```


Console commands:

```
ongps
21.10 20:14:46 time:181529 date:211121 lat:54.727939 lon:20.536888 sat:7 alt:63.0 spd:0.00 dir:1.28
21.10 20:14:47 time:181530 date:211121 lat:54.727939 lon:20.536888 sat:7 alt:63.0 spd:0.00 dir:1.28
21.10 20:14:47 time:181531 date:211121 lat:54.727939 lon:20.536886 sat:7 alt:63.0 spd:0.00 dir:1.28
21.10 20:14:48 time:181532 date:211121 lat:54.727939 lon:20.536884 sat:7 alt:62.9 spd:0.00 dir:1.28
21.10 20:14:49 time:181533 date:211121 lat:54.727939 lon:20.536884 sat:7 alt:62.9 spd:0.00 dir:1.28
21.10 20:14:50 time:181534 date:211121 lat:54.727939 lon:20.536882 sat:7 alt:62.9 spd:0.00 dir:1.28
21.10 20:14:51 time:181535 date:211121 lat:54.727939 lon:20.536878 sat:7 alt:62.9 spd:0.00 dir:1.28
21.10 20:14:52 time:181536 date:211121 lat:54.727939 lon:20.536878 sat:7 alt:62.9 spd:0.00 dir:1.28
21.10 20:14:53 time:181537 date:211121 lat:54.727939 lon:20.536878 sat:7 alt:62.9 spd:0.00 dir:1.28
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
19.10 21:31:13 Radio freq_list MHz:[101.8 102.5]
AT+FMCLOSE
OK

freemem
19.10 21:32:56 [StartDefaultTask] Free heap memory size 13416
```



P.S.

This project is under development and will be replenished.


