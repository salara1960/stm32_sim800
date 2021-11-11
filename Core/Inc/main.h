/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <malloc.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>

#include "cmsis_os.h"

#include "hdr.h"

//#include "usb_device.h"
//#include "usbd_cdc_if.h"

#include "func.h"

#ifdef SET_W25FLASH
	#include "w25.h"
#endif
#ifdef SET_OLED_I2C
	#include "ssd1306.h"
#endif
#ifdef SET_TEMP_SENSOR
	#include "ds18b20.h"
#endif
#ifdef SET_SMS
	#include "sms.h"
#endif


#include "gps.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum {
	devOK   = 0,
	devSPI  = 1,
	devUART = 2,
	devI2C  = 4,
	devRTC  = 8,
	devMem  = 0x10,
	devFifo = 0x20,
	devCDC  = 0x40,
	devFs   = 0x80,
	devGSM  = 0X100,
	devQue  = 0x200
};
enum {//INIT COMMANDS NUMBERS
	iAT = 0,
	iATE0,
	iCLTS,
	iCMGF,
	iCNMI,
	iGMR,
	iGSN,
	iCSQ,
	iCREG,
	iCGATT,
	iCIPSHUT,
	iCIPMODE,
	iCIPMUX
};
enum {//SNTP SERVER COMMANDS NUMERS
	//tCLBS = 0,
	tSAPBR = 0,
	tSAPBR31,
	tSAPBR11,
	tCNTP_CID,
	tCNTP_SRV,
	tCNTP,
	tCCLK,
	tSAPBR01
};
enum {
	nCIPSTATUS = 0,
	nCSTT,
	//{"AT+CIPSTATUS\r\n","OK"},//after OK -> STATE: IP START
	nCIICR,
	//{"AT+CIPSTATUS\r\n","OK"},//after OK -> STATE: IP GPRSACT
	nCIFSR,
	//{"AT+CIPSTATUS\r\n","OK"},//after OK -> STATE: IP STATUS
	nCIPSTART,
	nCIPSEND,
	//> QWERTY
	//SEND OK
	//qwerty
	nCIPCLOSE,
	nCIPSHUT
};
enum {
	fOPEN = 0,
	fVOLUME,
	fSCAN,
	fFREQ,
	fCLOSE
};
enum {
	cCUSD = 0,
	cCBC,
	cCCID,
	cCPIN,
	cCGATT,
	cATI,
	cCMEE,
	cCCLK
};
enum {
	_RDY = 0,
	_CFUN,
	_CPIN,
	_CallReady,
	_SMSReady,
	_Revision,
	_CSQ,
	_CREG,
	_CGATT,
	_CLBS,
	_CNTP,
	_CCLK,
	_CBC,
	_CMEE,
	_CUSD,
	_CMT,
	_SCLASS0,
	_STATE,
	_CONNECTOK,
	_ERROR,
	_OK
};

enum {
	seqInit = 0,
	seqTime,
	seqNet,
	seqRadio,
	seqAny
};

#define _10ms 1
#define _20ms (_10ms * 2)
#define _30ms (_10ms * 3)
#define _40ms (_10ms * 4)
#define _50ms (_10ms * 5)
#define _60ms (_10ms * 6)
#define _70ms (_10ms * 7)
#define _80ms (_10ms * 8)
#define _90ms (_10ms * 9)
#define _100ms (_10ms * 10)
#define _150ms (_10ms * 15)
#define _200ms (_10ms * 20)
#define _250ms (_10ms * 25)
#define _300ms (_10ms * 30)
#define _350ms (_10ms * 35)
#define _400ms (_10ms * 40)
#define _450ms (_10ms * 45)
#define _500ms (_10ms * 50)
#define _600ms (_10ms * 60)
#define _700ms (_10ms * 70)
#define _800ms (_10ms * 80)
#define _900ms (_10ms * 90)
#define _1s (_100ms * 10)
#define _1s5 (_100ms * 15)
#define _2s (_1s * 2)//2000
#define _2s5 (_100ms * 25)
#define _3s (_1s * 3)//3000
#define _4s (_1s * 4)//4000
#define _5s (_1s * 5)//5000
#define _10s (_1s * 10)//10000
#define _15s (_1s * 15)
#define _20s (_1s * 20)
#define _25s (_1s * 25)
#define _30s (_1s * 30)

#define APN      "internet"
#define LOGIN    "beeline"
#define PASSWORD "beeline"
#define SNTP     "pool.ntp.org"
#define TZONE    2
#define SRV_ADR  "213.149.17.142"
#define SRV_PORT 8778


#define CTRL_Z 0x1a

#define min_wait_ms 350
#define max_wait_ms 750

//#define MAX_SMS_BUF  384
#define MAX_FIFO_SIZE 48
#define MAX_GSM_BUF   512//768
#define MAX_GPS_BUF   128

#define CMD_LEN 40
#define ACK_LEN 32

#define CMD_REPEAT 5

#ifdef SET_OLED_I2C
	I2C_HandleTypeDef *portOLED;
	uint8_t i2cRdy;
#endif
#ifdef SET_W25FLASH
	SPI_HandleTypeDef *portFLASH;
	uint8_t spiRdy;
#endif

uint8_t devError;
const char *eol;

#pragma pack(push,1)
typedef struct {
	unsigned rdy:1;
	unsigned cFun:1;
	unsigned cPin:1;
	unsigned cReady:1;
	unsigned sReady:1;
	unsigned reg:1;
	unsigned cGat:1;
	unsigned cmee:2;
	unsigned tReady:1;
	unsigned reqDT:1;
	unsigned okDT:1;
	unsigned sms:1;
	unsigned state:2;
	unsigned connect:1;
	unsigned error:1;
	unsigned ok:1;
} gsmFlags_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	char cmd[CMD_LEN];
	char ack[ACK_LEN];
} ats_t;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct {//21/11/01,12:49:31+02
	int tz;
	int sec;
	int min;
	int hour;
	int year;
	int mon;
	int day;
} dattim_t;
#pragma pack(pop)



/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define LED_ERROR_Pin GPIO_PIN_0
#define LED_ERROR_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOB
#define DS18B20_Pin GPIO_PIN_12
#define DS18B20_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_11
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_12
#define GPS_RX_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define W25_CS_GPIO_Port SPI1_NSS_GPIO_Port
#define W25_CS_Pin SPI1_NSS_Pin

#ifdef SET_STATIC_MEM
	char PrnBuf[MAX_UART_BUF];// Служебный буфер для функции Report()
#endif

//#ifdef SET_TEMP_SENSOR
//	#define	_DS18B20_GPIO	DS18B20_GPIO_Port
//	#define	_DS18B20_PIN	DS18B20_Pin
//#endif
#ifdef USED_FREERTOS
	osSemaphoreId_t semHandle;
#endif

volatile bool setDate;
volatile uint32_t extDate;
UART_HandleTypeDef *portLOG;//порт логов (uart)
UART_HandleTypeDef *portGSM;//порт GSM модуля (sim800l)
I2C_HandleTypeDef  *portOLED;//порт OLED дисплея (ssd1306)
SPI_HandleTypeDef  *portFLASH;//порт flash-памяти (w25q64)
RTC_HandleTypeDef  *portRTC;
UART_HandleTypeDef *portGPS;//порт GPS модуля (ATGM332D)
TIM_HandleTypeDef *tmrDS18B20;


#define cmd_iniMax  13
#define cmd_timeMax  8
#define cmd_netMax  11
#define cmd_radioMax 5
#define cmd_anyMax   8
#define gsmEventMax 21
#define gsmStateMax  4

#define MAX_RSSI    32


const char *gsmState[gsmStateMax];
const char *gsmEvent[gsmEventMax];
char gsmREV[32];
char gsmIMEI[16];
char cntpSRV[64];
int8_t gsmRSSI;
uint16_t VCC;
char sntpDT[24];
const int8_t dBmRSSI[MAX_RSSI];

char *cusd;
dattim_t DT;


#ifdef SET_SMS
	#define len_From 32

	osSemaphoreId_t smsSem;

	const char *sim_auth_num;
	const char *sim_num;
	const char *dev_name;
	char fromNum[len_From];
	uint8_t abcd[5];
	uint16_t sms_num, sms_len;
	uint8_t sms_total;
	int8_t nrec;
	s_udhi_t reco;
	uint32_t wait_sms;
	bool smsFlag;
	s_recq_t smsq;
#endif

void *getMem(size_t len);
void freeMem(void *mem);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
