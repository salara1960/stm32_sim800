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
Start application version '1.9.3 (23.11.2021)'
23.11 15:12:49 [StartDefaultTask] Start main thread...(12904)
23.11 15:12:49 [StartTemp] Start sensor thread...(12904)
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
+CBC: 0,81,4049
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
+CCLK: "21/11/23,16:16:32+02"
Set date/time 23/11/21 16:16:32+02 OK !
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
10.224.127.127
AT+CIPSTATUS
OK
STATE: IP STATUS
AT+CIPSTART="TCP","91.109.152.236",8778
OK
CONNECT OK
AT+CIPSEND
>{"dev":"STM32_SIM800l","temp":26.75,"lat":54.7277,"lon":20.5368,"sat":7,"alt":33,"spd":0.00,"dir":230.47}
 
SEND OK
AT+CIPSEND
>{"dev":"STM32_SIM800l","temp":26.75,"lat":54.7277,"lon":20.5367,"sat":7,"alt":30,"spd":0.00,"dir":230.47}
 
SEND OK
.
.
.
AT+CIPSEND
>{"dev":"STM32_SIM800l","temp":26.81,"lat":54.7277,"lon":20.5368,"sat":6,"alt":28,"spd":0.46,"dir":54.40}
 
SEND OK
AT+CIPCLOSE
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
23.11 16:17:33 time:141733 date:231121 lat:54.727539 lon:20.536525 sat:6 alt:21.8 spd:3.06 dir:46.41
23.11 16:17:34 time:141734 date:231121 lat:54.727554 lon:20.536541 sat:6 alt:21.6 spd:3.61 dir:45.06
23.11 16:17:35 time:141735 date:231121 lat:54.727554 lon:20.536556 sat:6 alt:21.3 spd:3.06 dir:51.11
23.11 16:17:36 time:141736 date:231121 lat:54.727561 lon:20.536571 sat:6 alt:21.1 spd:3.08 dir:49.34
23.11 16:17:37 time:141737 date:231121 lat:54.727573 lon:20.536588 sat:6 alt:20.7 spd:3.01 dir:53.13
23.11 16:17:38 time:141738 date:231121 lat:54.727581 lon:20.536600 sat:6 alt:20.6 spd:3.01 dir:47.36
23.11 16:17:39 time:141739 date:231121 lat:54.727588 lon:20.536613 sat:6 alt:20.2 spd:3.10 dir:49.43
23.11 16:17:40 time:141740 date:231121 lat:54.727588 lon:20.536624 sat:6 alt:19.8 spd:2.76 dir:52.47
23.11 16:17:41 time:141741 date:231121 lat:54.727588 lon:20.536621 sat:6 alt:19.7 spd:0.00 dir:52.47
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
23.11 16:19:39 Radio freq_list MHz:[101.8 102.5]
AT+FMCLOSE
OK

freemem
23.11 16:20:04 [StartDefaultTask] Free heap memory size 13416
```



P.S.

This project is under development and will be replenished.


