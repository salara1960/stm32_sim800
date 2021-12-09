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
  - USART2 : parameters of port 115200 8N1 (transmit data with DMA) - a port for send AT command to SIM800l module.
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
Start application version '2.2 (09.12.2021)'
w25qxx Init Begin... Chip ID:0x4017
Chip W25Q64
        Page Size:      256 bytes
        Page Count:     32768
        Sector Size:    4096 bytes
        Sector Count:   2048
        Block Size:     65536 bytes
        Block Count:    128
        Capacity:       8192 KBytes
09.12 14:03:20 [StartDefaultTask] Start main thread...(6240)
09.12 15:03:20 [StartTemp] Start sensor thread...(6240)
09.12 15:03:20 Mount drive '0:/' OK
09.12 15:03:20 Read folder '/':
        name:CONF.TXT, size:22 bytes, attr:Archive
        name:CONF.CFG, size:22 bytes, attr:Archive
09.12 15:03:20 File '/conf.cfg' allready present
09.12 15:03:20 File 'conf.cfg' open OK
#Configuration file:

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
09.12 15:08:41 date:09/12/21 time:13:36:54 lat:54.727760 lon:20.534975 sat:7 alt:20.2 spd:2.15 dir:210.25
09.12 15:08:42 date:09/12/21 time:13:36:56 lat:54.727748 lon:20.534999 sat:7 alt:20.1 spd:0.83 dir:210.25
09.12 15:08:43 date:09/12/21 time:13:36:57 lat:54.727748 lon:20.534999 sat:7 alt:20.1 spd:0.83 dir:210.25
09.12 15:08:44 date:09/12/21 time:13:36:58 lat:54.727733 lon:20.535047 sat:8 alt:20.2 spd:1.20 dir:210.25
09.12 15:08:45 date:09/12/21 time:13:36:59 lat:54.727726 lon:20.535070 sat:8 alt:20.2 spd:1.25 dir:210.25
09.12 15:08:46 date:09/12/21 time:13:37:00 lat:54.727710 lon:20.535083 sat:8 alt:20.3 spd:0.86 dir:210.25
09.12 15:08:47 date:09/12/21 time:13:37:01 lat:54.727684 lon:20.535085 sat:8 alt:20.3 spd:1.38 dir:210.25
09.12 15:08:47 date:09/12/21 time:13:37:02 lat:54.727653 lon:20.535085 sat:8 alt:20.5 spd:2.76 dir:192.92
09.12 15:08:48 date:09/12/21 time:13:37:03 lat:54.727638 lon:20.535179 sat:8 alt:21.5 spd:1.63 dir:192.92
09.12 15:08:49 date:09/12/21 time:13:37:04 lat:54.727626 lon:20.535223 sat:8 alt:25.7 spd:0.59 dir:192.92
09.12 15:08:50 date:09/12/21 time:13:37:05 lat:54.727619 lon:20.535249 sat:8 alt:28.3 spd:0.79 dir:192.92
09.12 15:08:51 date:09/12/21 time:13:37:06 lat:54.727619 lon:20.535261 sat:8 alt:29.6 spd:0.68 dir:192.92
09.12 15:08:52 date:09/12/21 time:13:37:07 lat:54.727619 lon:20.535272 sat:8 alt:30.3 spd:0.37 dir:192.92
09.12 15:08:53 date:09/12/21 time:13:37:08 lat:54.727626 lon:20.535274 sat:8 alt:31.2 spd:0.62 dir:192.92
09.12 15:08:54 date:09/12/21 time:13:37:09 lat:54.727638 lon:20.535278 sat:9 alt:32.7 spd:0.53 dir:192.92
09.12 15:08:55 date:09/12/21 time:13:37:10 lat:54.727645 lon:20.535289 sat:9 alt:33.9 spd:0.83 dir:192.92
09.12 15:08:55 date:09/12/21 time:13:37:11 lat:54.727653 lon:20.535299 sat:10 alt:34.4 spd:0.00 dir:192.92
09.12 15:08:56 date:09/12/21 time:13:37:12 lat:54.727668 lon:20.535301 sat:10 alt:33.7 spd:0.00 dir:192.92
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

stop
AT+FMCLOSE
OK

rlist
09.12 15:10:23 Radio freq_list MHz:[101.8 102.5]

freemem
09.12 15:10:45 [StartDefaultTask] Free heap memory size 6152

read 0
Read sector:0 offset:0 len:1024
000000  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000020  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000040  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000060  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000080  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0000A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0000C0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0000E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000100  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000120  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000140  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000160  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000180  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0001A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01
0001C0  01 00 01 FE 3F 00 3F 00 00 00 C1 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0001E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 55 AA
000200  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000220  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000240  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000260  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000280  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0002A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0002C0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0002E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000300  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000320  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000340  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000360  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
000380  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0003A0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0003C0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
0003E0  00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

cat
09.12 15:11:07 File 'conf.cfg' open OK
#Configuration file:
```



P.S.

This project is under development and will be replenished.


