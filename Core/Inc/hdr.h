/*
 * hdr.h
 *
 *  Created on: Apr 25, 2021
 *      Author: alarm
 */

#define MAX_UART_BUF 896

#define SET_STATIC_MEM
//#define SET_CALLOC_MEM

#define SET_OLED_I2C

//#define SET_W25FLASH

#ifdef SET_W25FLASH
	//#define W25QXX_DEBUG
#endif

#define SET_FLOAT_PART

#define SET_GPS
#ifdef SET_GPS
	//#define SET_GPS_DEBUG
#endif

#define SET_TEMP_SENSOR

#define USED_FREERTOS

#define SET_SMS
#ifdef SET_SMS
	//#define SET_SMS_QUEUE
	//#define SET_RECQ_STATIC
#endif






