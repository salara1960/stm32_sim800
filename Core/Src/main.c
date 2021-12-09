/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

  arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 1792 * 4//2048 * 4
};
/* Definitions for tempTask */
osThreadId_t tempTaskHandle;
const osThreadAttr_t tempTask_attributes = {
  .name = "tempTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4//256 * 4
};
/* Definitions for rtcMutex */
osMutexId_t rtcMutexHandle;
const osMutexAttr_t rtcMutex_attributes = {
  .name = "rtcMutex"
};
/* Definitions for sem */
osSemaphoreId_t semHandle;
const osSemaphoreAttr_t sem_attributes = {
  .name = "sem"
};
/* USER CODE BEGIN PV */
//osMutexId_t rtcMutex;
//const osMutexAttr_t rtcMutex_attributes = {
//  .name = "rtcMutex"
//};


#ifdef SET_SMS
	#ifdef SET_SMS_QUEUE
		osSemaphoreId_t smsSem;
		const osSemaphoreAttr_t smsSem_attributes = {
			.name = "smsSen"
		};
	#endif
#endif

//const char *version = "0.1 (21.10.2021)";// first version
//const char *version = "0.2 (22.10.2021)";// todo fatfs
//const char *version = "0.3 (23.10.2021)";// todo fatfs (remove virtual serial port on usb device)
//const char *version = "0.4 (29.10.2021)";// add support AT commands for sim800l (first step)
//const char *version = "0.5 (30.10.2021)";// add specific queues
//const char *version = "0.6 (31.10.2021)";// add specific queues (item = struct)
//const char *version = "0.7 (01.11.2021)";// remove support fatfs
//const char *version = "0.8 (02.11.2021)";// move any functions to a separate file
//const char *version = "0.9 (03.11.2021)";// minor changes for parseEvent() function
//const char *version = "1.0 (04.11.2021)";// add cusd support and repeat command when error recv from GSM module
//const char *version = "1.1 (05.11.2021)";// add gps module (on uart6 + pps_pin with interrupt by rising/falling edge)
//const char *version = "1.2 (06.11.2021)";// add get date/time from SNTP server
//const char *version = "1.3 (07.11.2021)";// add get date/time from SNTP server
//const char *version = "1.3 (07.11.2021)";// set to RTC date/time received from SNTP server
//const char *version = "1.3.1 (07.11.2021)";// add tim10 and new Task - StartTemp() for ds18b20 sensor
//const char *version = "1.4 (10.11.2021)";// add support ds18b20 sensor
//const char *version = "1.5 (11.11.2021)";// support recv. sms messages (with concat parts of messages)
//const char *version = "1.5.1 (12.11.2021)";
//const char *version = "1.5.2 (14.11.2021)";
//const char *version = "1.5.3 (15.11.2021)";//minor changes for date/time set via sntp server
//const char *version = "1.6 (15.11.2021)";//major changes for get gps data
//const char *version = "1.7 (16.11.2021)";//major changes : remove gps thread and move nmea parser to main thread
//const char *version = "1.8 (17.11.2021)";
//const char *version = "1.8.1 (19.11.2021)";
//const char *version = "1.9 (20.11.2021)";
//const char *version = "1.9.1 (21.11.2021)";
//const char *version = "1.9.2 (22.11.2021)";//minor changes for sending json_data to external tcp server
//const char *version = "1.9.3 (23.11.2021)";
//const char *version = "1.9.4 (24.11.2021)"; //minor changes in +cusd parser and add local commands (via uart)
//const char *version = "1.9.5 (25.11.2021)";
//const char *version = "1.9.6 (26.11.2021)";
//const char *version = "1.9.7 (27.11.2021)";// Use parser answer from GSM module for enter AT commands via local channel (uart)
//const char *version = "2.0.1 (27.11.2021)";// add flash memory (w25q64)
//const char *version = "2.0.2 (30.11.2021)";//add reset flash_memory_chip (w25q64)
//const char *version = "2.0.3 (01.12.2021)";//add read 4 page from sector flash memory (w25q64)
//const char *version = "2.0.4 (02.12.2021)";//support local commands: read/write/erase sector from flash memory (w25q64)
//const char *version = "2.0.5 (02.12.2021)";//up speed for UART1 to 230400 and use read/write from/to flash (w25q64) via DMA
//const char *version = "2.1 (05.12.2021)";//project for STM32CubeIDE (first step for FatFs create/mount drive/open dir/mk-rd file)
//const char *version = "2.1.1 (08.12.2021)";
const char *version = "2.2 (09.12.2021)";// now FAT fs support (w25q54); add local command for cat conf.cfg file



volatile time_t epoch = 1639051400;//1638967160;//1638733115;//1638477416;//1638432926;
						//1638385311;//1638298187;//1638033160;//1637954401;//1637916982;//1637870245;//1637768795;//1637673169;
						//1637608799;//1637500605;//1637421807;//1637342030;//1637171390;//1637156150;//1637080774;//1637006802;
						//1636985372;//1636907840;//1636714630;//1636650430;//1636546510;//1636394530;//1636366999;//1636288627;
						//1636208753;//1636148268;//1636114042;//1636106564;//1636045527;//1636022804;//1635975820;//1635956750;
						//1635854199;//1635762840;//1635701599;//1635681180;//1635627245;//1635505880;//1635001599;//1634820289;

int tZone = TZONE;	// часовой пояс для установки даты и времени
volatile uint32_t cnt_err = 0;
//     Флаги событий
volatile uint8_t restart_flag = 0;
volatile uint8_t sntp_flag = 0;
volatile uint8_t ini_flag = 0;
volatile uint8_t radio_flag = 0;
//     Переменные для таймера (период 10 мсек)
volatile uint32_t secCounter = 0;
volatile uint32_t HalfSecCounter = 0;
//
volatile uint32_t extDate = 0;//unix timestamp
volatile bool setDate = false; //флаг установки даты и времени
//     шаблоны команд локального канала управления (uart)
const char *_extDate = "epoch=";
const char *_restart = "restart";
const char *_sntp = "sntp";
const char *_radio = "radio";
const char *_rlist = "rlist";
const char *_rnext = "rnext";
const char *_flags = "flags";
const char *_freemem = "free";
const char *_net  = "net";
const char *_ini  = "ini";
const char *_stop  = "stop";
const char *_clr  = "clr";
//     флаги команд локального канала управления (uart)
volatile bool prn_flags = false;
volatile bool prn_freemem = false;
volatile bool net_flag = false;
volatile bool prn_cmd = true;
volatile bool clr_flag = false;
//     Служебные переменные для для локального канала управления (uart)
char RxBuf[MAX_UART_BUF] = {0};// Буфер для приёма данных из порта portLOG
uint8_t rx_uk;
uint8_t uRxByte = 0;
uint8_t uartRdy = 1;
//     Служебные переменные для для канала управления GSM модулем (uart)
char gsmBuf[MAX_GSM_BUF] = {0};// Буфер для приёма данных из порта portGSM
uint16_t gsm_uk;
uint8_t gsmByte = 0;
uint8_t gsmRdy = 1;
uint8_t adone = 0;
//     Служебные переменные для для канала приема данных от модуля GPS (uart)
#ifdef SET_GPS
	UART_HandleTypeDef *portGPS = &huart6;//порт GPS модуля (ATGM332D)
	char gpsBuf[MAX_GPS_BUF] = {0};// Буфер для приёма данных из порта portGPS
	uint8_t gps_uk;
	uint8_t gpsByte = 0;
	uint8_t gpsRdy = 1;
	uint8_t gdone = 0;
	s_recq_t gpsFrom;
	bool gpsFromFlag = false;
	gsmFlags_t gpsFlags = {0};
	bool prnGpsFlag = false;
	// шаблоны команд локального канала управления (uart)
	const char *_ongps = "ongps";
	const char *_offgps = "offgps";
#endif

uint8_t *adrByte = NULL;

uint8_t devError = 0;//байт состояния устройства (позиционный код по всем интерфейсам)

//   Буфер для печати служебной информации (логи) через локальный канал управления
#ifdef SET_STATIC_MEM
	char PrnBuf[MAX_UART_BUF] = {0};// Служебный буфер для функции Report()
#endif
//    Указатели на структуры управления внутренними модулями сикроконтроллера
UART_HandleTypeDef *portLOG = &huart1;//порт логов (uart)
UART_HandleTypeDef *portGSM = &huart2;//порт GSM модуля (sim800l)
I2C_HandleTypeDef *portOLED = &hi2c1;//порт OLED дисплея (ssd1306)
SPI_HandleTypeDef *portFLASH = &hspi1;//порт flash-памяти (w25q64)
RTC_HandleTypeDef *portRTC = &hrtc;
#ifdef SET_TEMP_SENSOR
	TIM_HandleTypeDef *tmrDS18B20 = &htim10;
	float temp = 0.0;
#endif

//     Служебные переменные для внутренних модулей I2C, SPI
uint8_t i2cRdy = 1;
uint8_t screenON = 0;
uint32_t spi_cnt = 0;
uint8_t spiRdy = 1;
//
#ifdef SET_W25FLASH
	const char *_read  = "read";
	const char *_write  = "write";
	const char *_erase  = "erase";
	const char *_next  = "next";
	int adr_sector = 0, offset_sector = 0, list_sector = 0, len_write = 0;
	int cmd_sector = sNone, last_cmd_sector = sNone;
	uint8_t byte_write = 0xff;
	bool flag_sector = false;
	//
	#ifdef SET_FAT_FS
		FATFS FatFs;
		const char *cfg = "conf.cfg";
		bool mnt = false;
		const char *dirName = "/";
		bool cat_flag = false;
		const char *_cat  = "cat";
	#endif
	//
	unsigned char fs_work[_MAX_SS] = {0};
	char strf[1024] = {0};
	bool chipPresent = false;
	bool validChipID = false;
#endif
//      Служебные переменные для модулей GSM/GPRM, GPS/GLONASS
s_recq_t gsmTo;
bool gsmToFlag = false;
s_recq_t gsmFrom;
bool gsmFromFlag = false;
gsmFlags_t gsmFlags;
char gsmREV[32] = {0};
char gsmIMEI[16] = {0};
int8_t gsmRSSI = 0;
char cntpSRV[64] = {0};
uint16_t VCC = 0;
char sntpDT[24] = {0};
volatile bool gsmInit = false;
volatile bool sntpInit = false;
volatile bool netInit = false;
uint32_t packNumber = 0;

char *cmds = NULL;
char *cusd = NULL;
char *datas = NULL;

const char *eol = "\r\n";

const char *cmd_adr = NULL;

dattim_t DT;//date and time from sntp server

//     Служебные переменные для приема смс
#ifdef SET_SMS
	const char *sim_auth_num = "7XXXYYYZZZZ";
	const char *sim_num = "+79062103497";
	const char *dev_name = "STM32_SIM800l";
	char fromNum[lenFrom] = {0};
	uint8_t abcd[5] = {0};
	uint16_t sms_num, sms_len;
	uint8_t sms_total;
	int8_t nrec = -1;
	s_udhi_t reco;
	uint32_t wait_sms = 0;
	#ifdef SET_SMS_QUEUE
		bool smsFlag = false;
		s_recq_t smsq;
	#endif
#endif

//   Перменные для реализации режима RADIO у модуля SIM800l
uint8_t indList = 0;
uint16_t freqList[MAX_FREQ_LIST] = {0};
volatile bool prn_rlist = false;

//   Переменные для реализации периодической отправки данных на внешний сервер
uint32_t tmr_send = 0;
const uint32_t send_period = _10s;


//------------   AT команды GSM модуля   ------------------

const ats_t cmd_ini[cmd_iniMax] = {//INIT
		{"AT\r\n","OK"},
		{"ATE0\r\n","OK"},
		{"AT+CMEE=1\r\n","OK"},
		{"AT+CLTS=1\r\n","OK"},
		{"AT+CMGF=0\r\n","OK"},
		{"AT+CBC\r\n","OK"},
		{"AT+CNMI=1,2,0,1,0\r\n","OK"},
		{"AT+GMR\r\n","OK"},//Revision:1418B04SIM800L24
		{"AT+GSN\r\n","OK"},//864369032292264
		{"AT+CSQ\r\n","OK"},//+CSQ: 14,0
		{"AT+CREG?\r\n","OK"},//+CREG: 0,1
		{"AT+CGATT=1\r\n","OK"},
		{"AT+CIPMODE=0\r\n","OK"},
		{"AT+CIPMUX=0\r\n","OK"}
};
const ats_t cmd_time[cmd_timeMax] = {//GET TIME FROM SNTP SERVER
		{"AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n","OK"},
		{"AT+SAPBR=3,1,\"APN\",","OK"},//\"internet\"\r\n" | "APN" + eol
		{"AT+SAPBR=1,1\r\n","OK"},
		{"AT+CNTPCID=1\r\n","OK"},
		{"AT+CNTP=","OK"},//pool.ntp.org\",8\r\n"        | "SNTP",TZONE<<2 + eol
		{"AT+CNTP\r\n","+CNTP: "},//+CNTP: 1
		{"AT+CCLK?\r\n","OK"},//+CCLK: "21/11/01,12:49:31+02"
		{"AT+SAPBR=0,1\r\n","OK"}
};
const ats_t cmd_net[cmd_netMax] = {//SET CONTEXT AND MAKE CONNECTION
		{"AT+CIPSTATUS\r\n","OK"},//after OK -> STATE: IP INITIAL
		{"AT+CSTT=","OK"},//\"internet\",\"beeline\",\"beeline\"\r\n","OK"}, | "APN","LOGIN","PASSWORD" + eol
		{"AT+CIPSTATUS\r\n","STATE: "},//after OK -> STATE: IP START
		{"AT+CIICR\r\n","OK"},
		{"AT+CIPSTATUS\r\n","STATE: "},//after OK -> STATE: IP GPRSACT
		{"AT+CIFSR\r\n",""},//10.206.118.240
		{"AT+CIPSTATUS\r\n","STATE: "},//after OK -> STATE: IP STATUS
		{"AT+CIPSTART=\"TCP\",","CONNECT OK"}//\"213.149.17.142\",8778\r\n","OK"}//after OK -> CONNECT OK | "SRV"
		//
//		{"AT+CIPSEND\r\n",">"},//send data after '>'
		//> QWERTY
		//SEND OK
		//qwerty
		//
//		{"AT+CIPCLOSE\r\n","CLOSE OK"},
//		{"AT+CIPSHUT\r\n","SHUT OK"}
};
const ats_t cmd_radio[cmd_radioMax] = {
		{"AT+FMOPEN=0\r\n","OK"},//   ; вкыл
		{"AT+FMVOLUME=6\r\n","OK"},//6 ;громкость от 0 до 6           | 6 + eol
		{"AT+FMSCAN\r\n","OK"},// list of founded freq
		{"AT+FMFREQ=","OK"}//,//1025 ; установить чатоту 102.5 Мгц | 1025 + eol
};
const ats_t cmd_any[cmd_anyMax] = {
		{"AT+CUSD=1,","OK"},//"#102#"" | "#102#" + eol    | +CUSD: 0, " Vash balans 200.00 r.", 15
		{"AT+CCID\r\n","OK"},//8970199181027011035f
		{"AT+CPIN?\r\n","OK"},//+CPIN: READY
		{"AT+CGATT?\r\n","OK"},//+CGATT: 1
		{"ATI\r\n","OK"},//SIM800 R14.18
		{"AT+CMEE?\r\n","OK"},//0 or 1
		{"AT+CCLK?\r\n","OK"},//+CCLK: "21/11/01,12:49:31+02"
		{"AT+CIPCLOSE\r\n","CLOSE OK"},
		{"AT+CIPSHUT\r\n","SHUT OK"},
		{"AT+CIPSEND\r\n",">"},//send data after '>'
		{"AT+FMCLOSE\r\n","OK"}
};

//------------   События от GSM модуля   ------------------

const char *gsmEvent[gsmEventMax] = {
		"RDY",
		"+CFUN: ",//1
		"+CPIN: ",//READY
		"Call Ready",
		"SMS Ready",
		"Revision:",//1418B04SIM800L24
		"+CSQ: ",//14,0
		"+CREG: ",//0,1
		"+CGATT: ",//1
		"+CLBS: ",//3
		"+CNTP: ",//1
		"+CCLK: ",//"21/11/01,12:49:31+02"
		"+CBC: ",//0,73,3988
		"+CMEE: ",//1
		"+CUSD: ",//+CUSD: 0, "003200300030002E003000300020 .... 340023", 72
		"+CMT: ",
		"+SCLASS0: ",
		"STATE: ",//"IP INITIAL","IP START","IP GPRSACT","IP STATUS"
		"CONNECT OK",
		"CONNECT FAIL",
		"CLOSE",
		"SHUT OK",
		">",
		"SEND OK",
		"ERROR",
		"OK"
};

const char *gsmState[gsmStateMax] = {
		"IP INITIAL",
		"IP START",
		"IP GPRSACT",
		"IP STATUS",
		"TCP CLOSED",
		"???"
};

const int8_t dBmRSSI[MAX_RSSI] = {
	-113,-111,-109,-107,-105,-103,-101,-99,
	-97 ,-95 ,-93 ,-91 ,-89 ,-87 ,-85 ,-83,
	-81 ,-79 ,-77 ,-75 ,-73 ,-71 ,-69 ,-67,
	-65 ,-63 ,-61 ,-59 ,-57 ,-55 ,-53 ,-51
};

int cmdID = -1;
int evtID = -1;

osStatus_t coreStatus = osError;

const cmd_info_t cmd_info[MAX_CMD_INFO] = {
	{&cmd_ini[0].cmd[0],   seqInit,  cmd_iniMax},
	{&cmd_time[0].cmd[0],  seqTime,  cmd_timeMax},
	{&cmd_net[0].cmd[0],   seqNet,   cmd_netMax},
	{&cmd_radio[0].cmd[0], seqRadio, cmd_radioMax},
	{&cmd_any[0].cmd[0],   seqAny,   cmd_anyMax}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void *argument);
void StartTemp(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  	  //   мигнем тремя светодиодами
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(350);
      HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(350);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_SET);


      // start timer3 in interrupt mode
      HAL_TIM_Base_Start_IT(&htim3);
      // start timer10 in interrupt mode
      HAL_TIM_Base_Start_IT(&htim10);

      //"start" rx_interrupt
      HAL_UART_Receive_IT(portLOG, (uint8_t *)&uRxByte, 1);
      //"start" rx_gsm_interrupt
      HAL_UART_Receive_IT(portGSM, (uint8_t *)&gsmByte, 1);
#ifdef SET_GPS
      //"start" rx_gps_interrupt
      HAL_UART_Receive_IT(portGPS, (uint8_t *)&gpsByte, 1);
#endif

      Report(NULL, true, "Start application version '%s'\r\n", version);

      cmds = (char *)calloc(1, CMD_LEN + 1);//выделяем память под буфер команд GSM модуля
#ifdef SET_SMS
      cusd = (char *)calloc(1, SMS_BUF_LEN);//выделяем память под буфер для приема ответа на cusd-запросы

      InitSMSList();// инициализация структуры для хранения частей смс
#endif


#ifdef SET_W25FLASH

      //HAL_Delay(1000);

      chipPresent = W25qxx_Init();
      uint32_t cid = W25qxx_getChipID();
      if ( chipPresent && ((cid >= W25Q10) && (cid <= W25Q128)) ) validChipID = true;
      list_sector = W25qxx_getPageSize() << 2;

#ifdef SET_FAT_FS
      /*
      mnt = drvMount(USERPath);
      if (mnt) {
    	  //------------------------------------------------------------------------
    	  dirList(dirName);
    	  //
    	  sprintf(strf, "#Configuration file:%s", eol);
    	  mkFile(cfg, strf);
    	  //
    	  rdFile(cfg);
    	  //------------------------------------------------------------------------
      }*/
      /*if (dir_open) {
    	  f_closedir(&dir);
    	  dir_open = false;
    	  Report(NULL, true, "Close dir '%s'%s", ps, eol);
      }
      if (mnt) {
    	  f_mount(NULL, USERPath, 1);
    	  mnt = false;
    	  Report(NULL, true, "Umount drive '%.*s'%s", sizeof(USERPath), USERPath, eol);
      }*/
#endif

	  if (devError) errLedOn(NULL);


      //---------------------------------------------------
#endif

      //-------------------------------------------------
      //     Инициализация OLED дисплея
      oled_withDMA = 1;
      screenON = 1;
      i2c_ssd1306_init();//screen INIT
      i2c_ssd1306_pattern(false);//set any params for screen
      //i2c_ssd1306_invert();
      //i2c_ssd1306_scroll(1, 8, OLED_CMD_SHIFT_STOP);
      i2c_ssd1306_clear();//clear screen

      //---------------------------------------------------


//LOOP_FOREVER();


      //coreStatus = osKernelInitialize();

  /* USER CODE END 2 */

  /* Init scheduler */
      coreStatus = osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of rtcMutex */
  rtcMutexHandle = osMutexNew(&rtcMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem */
  semHandle = osSemaphoreNew(1, 1, &sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  //mainBinSemHandle = osSemaphoreNew(1, 1, &mainBinSem_attributes);
#ifdef SET_SMS
	#ifdef SET_SMS_QUEUE
  		smsFlag = initSRECQ(&smsq);

  		smsSem = osSemaphoreNew(1, 1, &smsSem_attributes);
	#endif
#endif
  		//rtcMutexHandle = osMutexNew(&rtcMutex_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  //if (!gsmQueToHandle || !gsmQueFromHandle) devError |= devQue;

  	//    Инициализация служебных очередей для работы с GSM модулем
  	gsmToFlag   = initRECQ(&gsmTo);
  	gsmFromFlag = initRECQ(&gsmFrom);
#ifdef SET_GPS
  	//    Инициализация служебной очереди для работы с GPS модулем
  	gpsFromFlag = initRECQ(&gpsFrom);
#endif

  	//   Начальная установка в RTC даты и времени из переменной epoch (unix tamestamp)
  	set_Date((time_t)epoch);//set time last edit

  /* add queues, ... */
  //"start" rx_gsm_interrupt
  //char *adr = NULL;
  //gsmQueFrom = osMessageQueueNew(16, sizeof(adr), &gsmQueFrom_attr);
  //gsmQueTo   = osMessageQueueNew(16, sizeof(uint16_t), &gsmQueTo_attr);

  //---------------------------------------------------

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartDefaultTask, NULL, &mainTask_attributes);

  /* creation of tempTask */
  tempTaskHandle = osThreadNew(StartTemp, NULL, &tempTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 41999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;//115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_ERROR_Pin|LED_Pin|CON_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_ERROR_Pin LED_Pin CON_LED_Pin */
  GPIO_InitStruct.Pin = LED_ERROR_Pin|LED_Pin|CON_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef SET_W25FLASH
//
#ifdef SET_FAT_FS
//------------------------------------------------------------------------------------
static char *fsErrName(int fr)
{
	switch (fr) {
		case FR_OK:				// (0) Succeeded
			return "Succeeded";
		case FR_DISK_ERR://			(1) A hard error occurred in the low level disk I/O layer
			return "Error disk I/O";
		case FR_INT_ERR://			(2) Assertion failed
			return "Assertion failed";
		case FR_NOT_READY://		(3) The physical drive cannot work
			return "Drive not ready";
		case FR_NO_FILE://			(4) Could not find the file
			return "No file";
		case FR_NO_PATH://			(5) Could not find the path
			return "No path";
		case FR_INVALID_NAME://		(6) The path name format is invalid
			return "Path error";
		case FR_DENIED://			(7) Access denied due to prohibited access or directory full
		case FR_EXIST://			(8) Access denied due to prohibited access
			return "Access denied";
		case FR_INVALID_OBJECT://	(9) The file/directory object is invalid
			return "Invalid file/dir";
		case FR_WRITE_PROTECTED://	(10) The physical drive is write protected
			return "Write protected";
		case FR_INVALID_DRIVE://	(11) The logical drive number is invalid
			return "Invalid drive number";
		case FR_NOT_ENABLED://		(12) The volume has no work area
			return "Volume no area";
		case FR_NO_FILESYSTEM://	(13) There is no valid FAT volume
			return "Volume has't filesystem";
		case FR_MKFS_ABORTED://		(14) The f_mkfs() aborted due to any problem
			return "f_mkfs() aborted";
		case FR_TIMEOUT://			(15) Could not get a grant to access the volume within defined period
			return "Timeout access";
		case FR_LOCKED://			(16) The operation is rejected according to the file sharing policy
			return "File locked";
		case FR_NOT_ENOUGH_CORE://	(17) LFN working buffer could not be allocated
			return "Allocated buf error";
		case FR_TOO_MANY_OPEN_FILES://	(18) Number of open files > _FS_LOCK
			return "Open file limit";
		case FR_INVALID_PARAMETER://	(19) Given parameter is invalid
			return "Invalid parameter";
	}
	return "Unknown error";
}
//------------------------------------------------------------------------------------------
static char *attrName(uint8_t attr)
{
	switch (attr) {
		case AM_RDO://	0x01	/* Read only */
			return "Read only";
		case AM_HID://	0x02	/* Hidden */
			return "Hidden";
		case AM_SYS://	0x04	/* System */
			return "System";
		case AM_DIR://	0x10	/* Directory */
			return "Directory";
		case AM_ARC://	0x20	/* Archive */
			return "Archive";
		default : return "Unknown";
	}
}
//------------------------------------------------------------------------------------------
bool drvMount(const char *path)
{
bool ret = false;

	if (!validChipID) return ret;

	FRESULT res = f_mount(&FatFs, path, 1);
	if (res == FR_NO_FILESYSTEM) {
		Report(NULL, true, "Mount drive '%s' error #%u (%s)%s", path, res, fsErrName(res), eol);
		res = f_mkfs(path, FM_FAT, W25qxx_getBlockSize(), fs_work, sizeof(fs_work));
		if (!res) {
			Report(NULL, true, "Make FAT fs on drive '%s' OK%s", path, eol);
			res = f_mount(&FatFs, path, 1);
    	} else {
    		Report(NULL, true, "Make FAT fs error #%u (%s)%s", res, fsErrName(res), eol);
    	}
	}
	if (!res) {
		ret = true;
		Report(NULL, true, "Mount drive '%s' OK%s", path, eol);
	} else {
		Report(NULL, false, "Mount drive '%s' error #%u (%s)%s", path, res, fsErrName(res), eol);
	}

	return ret;
}
//--------------------------------------------------------------------------------------------------------
void dirList(const char *name_dir)
{
DIR dir;

	FRESULT res = f_opendir(&dir, name_dir);
	if (!res) {
		FILINFO fno;
		int cnt = -1;
		Report(NULL, true, "Read folder '%s':%s", name_dir, eol);
		for (;;) {
			res = f_readdir(&dir, &fno);
			cnt++;
			if (res || fno.fname[0] == 0) {
				if (!cnt) Report(NULL, false, "\tFolder '%s' is empty%s", name_dir, eol);
				break;
			} else if (fno.fattrib & AM_DIR) {// It is a directory
				Report(NULL, false, "\tIt is folder -> '%s'%s", fno.fname, eol);
			} else {// It is a file.
				Report(NULL, false, "\tname:%s, size:%u bytes, attr:%s%s",
									fno.fname,
									fno.fsize,
									attrName(fno.fattrib),
									eol);
			}
		}
		f_closedir(&dir);
	}
}
//--------------------------------------------------------------------------------------------------------
void wrFile(const char *name, char *text, bool update)
{
char tmp[128];
FIL fp;
FRESULT res = FR_NO_FILE;

	sprintf(tmp, "/%s", cfg);
	if (!update) {
		res = f_open(&fp, tmp, FA_READ);
		if (res == FR_OK) {
			res = f_close(&fp);
			Report(NULL, true, "File '%s' allready present%s", tmp, eol);
			return;
		}
	}

	res = f_open(&fp, tmp, FA_CREATE_NEW | FA_WRITE);
	if (!res) {
		Report(NULL, true, "Create new file '%s' OK%s", tmp, eol);
		int wrt = 0, dl = strlen(text);

		wrt = f_puts(text, &fp);
		if (wrt != dl) {
			devError |= devFS;
			Report(NULL, true, "Error while write file '%s'%s", tmp, eol);
		} else Report(NULL, true, "File file '%s' write OK%s", tmp, eol);

		res = f_close(&fp);
	} else Report(NULL, true, "Create new file '%s' error #%u (%s)%s", tmp, res, fsErrName(res), eol);

}
//--------------------------------------------------------------------------------------------------------
void rdFile(const char *name)
{
char tmp[128];
FIL fp;

	if (!f_open(&fp, name, FA_READ)) {
		Report(NULL, true, "File '%s' open OK%s", name, eol);

		while (f_gets(tmp, sizeof(tmp) - 1, &fp) != NULL)
			Report(NULL, false, "%s", tmp);

		f_close(&fp);
	} else Report(NULL, true, "Error while open for reading file '%s'%s", name, eol);

}
//--------------------------------------------------------------------------------------------------------
#endif
//
#endif
/*
//------------------------------------------------------------------------------------
//             Функция выделяет из "кучи" блок памяти
//
void *getMem(size_t len)
{
#ifdef SET_CALLOC_MEM
		return (calloc(1, len));
#else
	#ifdef SET_MALLOC_MEM
		return (malloc(len));
	#else
		return (pvPortMalloc(len));
	#endif
#endif
}
//------------------------------------------------------------------------------------
//             Функция освобождает блок памяти
//
void freeMem(void *mem)
{
#if defined(SET_CALLOC_MEM) || defined(SET_MALLOC_MEM)
		free(mem);
#else
		vPortFree(mem);
#endif
}
*/
//------------------------------------------------------------------------------------------
//   Функция формирует символьное сообщения согласно кода ошибки
//
void errNameStr(uint8_t er, char *st)
{
	*st = '\0';

	if (er & devSPI)  strcat(st, "SPI");//case devSPI://  = 1,
	if (er & devUART) strcat(st, " UART");// = 2,
	if (er & devI2C)  strcat(st, " I2C");//  = 4,
	if (er & devRTC)  strcat(st, " RTC");//  = 8,
	if (er & devMem)  strcat(st, " Mem");//  = 0x10,
	if (er & devGSM)  strcat(st, " GSM");//  = 0X20,
	if (er & devQue)  strcat(st, " Que");// = 0x40
#ifdef SET_GPS
	if (er & devCRC)  strcat(st, " CRC");//  = 0x100
#endif
	if (!strlen(st)) sprintf(st,"err : 0x%02X", er);
}
//------------------------------------------------------------------------------------------
//             Функция обработки принятых данных с порта логов (portLOG)
//
void serialLOG()
{
	char *uki = NULL;

	if (uRxByte != BACK_SPACE)
		RxBuf[rx_uk & 0xff] = (char)uRxByte;
	else
		if (rx_uk) rx_uk--;

	if (uRxByte == 0x0a) {//end of line
		bool mk_at = false;
		char *uk = strstr(RxBuf, _extDate);//const char *_extDate = "epoch=";
		if (uk) {
			uk += strlen(_extDate);
			if (*uk != '?') {
				if (strlen(uk) < 10) setDate = false;
				else {
					char *uke = strchr(uk, ':');
					if (uke) {
						tZone = atoi(uke + 1);
						*uke = '\0';
					} else tZone = 0;
					extDate = atol(uk);
					//if (tZone) extDate += (tZone * 60);
					set_Date((time_t)extDate);
				}
			} else setDate = true;
		}
#ifdef SET_FAT_FS
		else if (!strncmp(RxBuf, _cat, strlen(_cat))) {
			if (!cat_flag && !gsmFlags.busy) cat_flag = 1;
		}
#endif
		else if (!strncmp(RxBuf, _restart, strlen(_restart))) {
			if (!restart_flag) restart_flag = 1;
		} else if ( (strstr(RxBuf, "AT")) || (strstr(RxBuf, "at")) ) {// enter AT_commands
			if (gsmToFlag) {
				uk = strchr(RxBuf, '\n');
				if (uk) *(uk) = '\0';
				else {
					uk = strchr(RxBuf, CTRL_Z);
					if (uk) *(uk + 1) = '\0';
				}
				strcat(RxBuf, eol);
				prn_cmd = false;
				mk_at = true;
			}
		} else if (!strncmp(RxBuf, _flags, strlen(_flags))) {
			if (!prn_flags) prn_flags = true;
		} else if (!strncmp(RxBuf, _sntp, strlen(_sntp))) {
			if (!sntp_flag && !gsmFlags.busy) sntp_flag = 1;
		} else if (!strncmp(RxBuf, _radio, strlen(_radio))) {
			if (!radio_flag && !gsmFlags.busy) radio_flag = 1;
		} else if (!strncmp(RxBuf, _rlist, strlen(_rlist))) {
			if (!prn_rlist) prn_rlist = true;
		} else if (!strncmp(RxBuf, _freemem, strlen(_freemem))) {
			if (!prn_freemem) prn_freemem = true;
		} else if (!strncmp(RxBuf, _net, strlen(_net))) {
			if (!net_flag && !gsmFlags.busy) net_flag = true;
		} else if (!strncmp(RxBuf, _ini, strlen(_ini))) {
			if (!ini_flag && !gsmFlags.busy) ini_flag = 1;
		}else if (!strncmp(RxBuf, _stop, strlen(_stop))) {
			if (gsmFlags.connect) {
				strcpy(RxBuf, cmd_any[cCLOSE].cmd);
				prn_cmd = true;
				mk_at = true;
			} else if (gsmFlags.play) {
				gsmFlags.play = 0;
				strcpy(RxBuf, cmd_any[cFMCLOSE].cmd);
				prn_cmd = true;
				mk_at = true;
			}
		} else if (!strncmp(RxBuf, _clr, strlen(_clr))) {
			clr_flag = true;
		}
#ifdef SET_W25FLASH
		else if (!strncmp(RxBuf, _read, strlen(_read))) {
			cmd_sector = sRead;
			uki = RxBuf + strlen(_read);
		} else if (!strncmp(RxBuf, _write, strlen(_write))) {
			cmd_sector = sWrite;
			uki = RxBuf + strlen(_write);
		} else if (!strncmp(RxBuf, _erase, strlen(_erase))) {
			cmd_sector = sErase;
			uki = RxBuf + strlen(_erase);
		} else if (!strncmp(RxBuf, _next, strlen(_next))) {
			cmd_sector = sNext;
		}
		if ((cmd_sector != sNone) && validChipID) {
			uk = strchr(RxBuf, '\n');
			if (uk) *(uk) = '\0';
			switch (cmd_sector) {
				case sRead:// read 0
				case sErase:// erase 0
					if (*uki == ' ') {
						int sek = atoi(++uki);
						if ((sek >= 0) && (sek < W25qxx_getSectorCount())) {
							adr_sector = sek;
							offset_sector = 0;
							flag_sector = true;
						}
					}
				break;
				case sWrite:// write 0 a5 | write 0 a5 256
					if (*uki == ' ') {
						uki++;
						int sek = atoi(uki);
						if ((sek >= 0) && (sek < W25qxx_getSectorCount())) {
							char *ukn = strchr(uki, ' ');
							if (ukn) {
								len_write = -1;
								ukn++;
								byte_write = hexToBin(ukn);
								uki = strchr(ukn, ' ');
								if (uki) {
									int l = atoi(++uki);
									if ((l > 0) && (l < W25qxx_getSectorSize())) len_write = l;
								}
								adr_sector = sek;
								offset_sector = 0;
								flag_sector = true;
							}
						}
					}
				break;
				case sNext:// next
					if ((last_cmd_sector == sRead) || (last_cmd_sector == sNext)) {
						if ((offset_sector + list_sector) < W25qxx_getSectorSize()) {
							offset_sector += list_sector;
							flag_sector = true;
						}
					}
				break;
			}
		}
#endif
#ifdef SET_GPS
		else if (!strncmp(RxBuf, _ongps, strlen(_ongps))) {
			if (!prnGpsFlag) prnGpsFlag = true;
		} else if (!strncmp(RxBuf, _offgps, strlen(_offgps))) {
			if (prnGpsFlag) prnGpsFlag = false;
		}
#endif
		// блок помещает в очередь команд новое сообщение для GSM модуля
		if (mk_at) {
			char *to = (char *)calloc(1, strlen(RxBuf) + 1);
			if (to) {
				strcpy(to, RxBuf);
				toUppers(to);
				if (putRECQ(to, &gsmTo) < 0) {
					devError |= devQue;
					free(to);
				} else {
					if (devError & devQue) devError &= ~devQue;
				}
			} else devError |= devMem;
		}
		//--------------------------------------------------------------

		rx_uk = 0;
		memset(RxBuf, 0, sizeof(RxBuf));

	} else rx_uk++;

}
//------------------------------------------------------------------------------------------
//          Функция обработки принятых данных с порта GSM модуля (portGSM)
//
void serialGSM()
{
	if ((gsmByte > 0x0d) && (gsmByte < 0x80)) {
		if (gsmByte >= 0x20) adone = 1;
		if (adone) gsmBuf[gsm_uk++] = (char)gsmByte;
	}
	if (adone) {
		if ( (gsmByte == 0x0a) || (gsmByte == 0x3e) ) {// '\n' || '>'
			if (gsmByte != 0x3e) strcat(gsmBuf, "\r\n");//0x0D 0x0A
			int len = strlen(gsmBuf);
			if (gsmFromFlag) {
				// Блок помещает в очередь ответов на команду очередное сообщение от модуля GSM
				char *from = (char *)calloc(1, len + 1);
				if (from) {
					memcpy(from, gsmBuf, len);
					if (putRECQ(from, &gsmFrom) < 0) {
						devError |= devQue;
						free(from);
					} else {
						if (devError & devQue) devError &= ~devQue;
					}
				} else {
					devError |= devMem;
				}
				//-----------------------------------------------------------------------------
			}

			gsm_uk = 0;
			adone = 0;
			memset(gsmBuf, 0, sizeof(gsmBuf));
		}
	}
}
//------------------------------------------------------------------------------------------
//          Функция обработки принятых данных с порта GPS модуля (portGPS)
//
#ifdef SET_GPS
void serialGPS()
{
	if ((gpsByte > 0x0d) && (gpsByte < 0x80)) {
		if (gpsByte >= 0x24) gdone = 1; //'$' - first symbol in nmea message
		if (gdone) gpsBuf[gps_uk++] = (char)gpsByte;
	}
	if (gdone) {
		if (gpsByte == 0x0a) {// '\n'
			gpsBuf[gps_uk] = '\0';
			if (gpsFromFlag) {// && gsmInit && sntpInit)
				if (gpsValidate(gpsBuf)) {// Функция проверяет на валиднсть данные от GPS модуля
					//char *uk = strstr(gpsBuf, eol);
					//if (uk) *uk = '\0';
					int len = strlen(gpsBuf);
					char *gf = (char *)calloc(1, len + 1);
					if (gf) {
						// Блок помещает в очередь данные, полученные от GPS модуля
						memcpy(gf, gpsBuf, len);
						if (putRECQ(gf, &gpsFrom) < 0) {
							devError |= devQue;
							free(gf);
						} else {
							if (devError & devQue) devError &= ~devQue;
						}
						//---------------------------------------------------------
					} else devError |= devMem;
				}
			}
			gps_uk = 0;
			gdone = 0;
			memset(gpsBuf, 0, sizeof(gpsBuf));
		}
	}
}
//-------------------------------------------------------------------------------------------
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPS_PPS_Pin) HAL_GPIO_TogglePin(LED_PPS_GPIO_Port, LED_PPS_Pin);
}
*/
#endif
//------------------------------------------------------------------------------------------
//                      CallBack функция портов UART,
//         вызывается по завершении передачи данных в порт UART
//
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) uartRdy = 1;
	else
	if (huart->Instance == USART2) gsmRdy = 1;
#ifdef SET_GPS
	else
	if (huart->Instance == USART6) gpsRdy = 1;
#endif
}
//------------------------------------------------------------------------------------------
//                      CallBack функция портов UART,
//            вызывается по завершении приема данных из порта UART
//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	adrByte = NULL;

	if (huart->Instance == USART1) {
		serialLOG();
		adrByte = &uRxByte;
	} else if (huart->Instance == USART2) {
		serialGSM();
		adrByte = &gsmByte;
	}
#ifdef SET_GPS
	else if (huart->Instance == USART6) {
		serialGPS();
		adrByte = &gpsByte;
	}
#endif

	HAL_UART_Receive_IT(huart, adrByte, 1);
}
//-------------------------------------------------------------------------------------------
//    CallBack функция, вызывается при возникновении ошибки у модуля I2C
//
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) devError |= devI2C;
}
//------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении передачи данных в порт модуля I2C
//
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) i2cRdy = 1;//from oled
}
/*
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
*/
//------------------------------------------------------------------------------------------
//    Функция устанавливает в начальное состояние 'готов' служебные параметры порта SPI1:
//
void spiDone(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1) {
#ifdef SET_W25FLASH
		W25_UNSELECT();
		spiRdy = 1;
#endif
	}
}
//------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении приема данных от порта SPI
//
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
}
//------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении передачи данных в порт SPI
//
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
}
//------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении приема/передачи данных порта SPI
//
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
}
//-------------------------------------------------------------------------------------------
//    CallBack функция, вызывается при возникновении ошибки у модуля SPI
//
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
	devError |= devSPI;
}
//-------------------------------------------------------------------------------------------

/*
//-----------------------------------------------------------------------------
//__STATIC_FORCEINLINE unsigned int nmeaCRC(char *uk, uint32_t len)
inline unsigned int nmeaCRC(char *uk, uint32_t len)
{
unsigned int res = 0;

	__asm volatile ("push {r0,r1,r2,r3}\n\t"
					"ldr r1, %1\n\t"
                    "ldr r2, %2\n\t"
					"ldr r0, #0\n\t"
					"loop: ldrb r3, [r1]\n\t"
                    "eor r0, r3\n\t"
					"and r0, #255\n\t"
					"add r1, #1\n\t"
					"subs r2, #1\n\t"
					"cbnz r2, loop\n\t"
					"str r0, %0\n\t"
			        "pop {ro,r1,r2,r3}" : "=r" (res) : "r" (uk), "r" (len) : "r0");

	return res;
}
//-----------------------------------------------------------------------------
*/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	//---------------------------------------------------------------
	//
	// Нитка обработки команд, событий и данных от GSM и GPS модулей
	//
	//---------------------------------------------------------------

	Report(__func__, true, "Start main thread...(%lu)\r\n", xPortGetFreeHeapSize());

    //---------------------------------------------------------------
/**/
#ifdef SET_FAT_FS

	osDelay(500);

    mnt = drvMount(USERPath);
    if (mnt) {
    	//------------------------------------------------------------------------
    	dirList(dirName);
    	//
    	sprintf(strf, "#Configuration file:%s", eol);
    	wrFile(cfg, strf, false);
    	//
    	rdFile(cfg);
    	//------------------------------------------------------------------------
    }
#endif
/**/

    //---------------------------------------------------------------
//#ifdef SET_GPS
      /*
      static char nmeaTest[96];
      strcpy(nmeaTest, "$GNGGA,163522.000,5443.66276,N,02032.21629,E,1,06,3.1,-5.0,M,0.0,M,,*57");
      char vrem[MAX_GSM_BUF] = {0};
      //
      uint32_t u_crc = 0;
      char *u_start = &nmeaTest[1];
      char *u_end = strchr(u_start, '*');
      if (u_end) {
    	  //u_crc = nmeaCRC(u_start, (uint32_t)(u_end - u_start));
    	  //sprintf(vrem, "CRC Test: '%s' - 0x%X", nmeaTest, u_crc);
    	  //Report(NULL, false, "%s%s", vrem, eol);
      }
      */
//#endif
	//---------------------------------------------------------------

	//   Описание и инициализация служебных переменных
	char *uks = NULL;
	char buf[MAX_GSM_BUF] = {0};
	char buf2[MAX_GSM_BUF] = {0};
	char scr[MAX_GSM_BUF] = {0};
	char cmdBuf[MAX_CMD_BUF] = {0};
	int8_t slin = 3, elin = 4, clin = 3, at_line = 2, err_line = 8, temp_line = 5;

	int8_t cur_cmd = -1;
	int8_t grp_cmd = -1;
	uint8_t cmd_err = CMD_REPEAT;
	const char *uk_cmd = NULL;
	const char *uk_ack = NULL;
	bool next_cmd = true;
	bool at_auto = false;
	uint32_t tmr_cmd = get_tmr(5);
	uint32_t tmr_next = 0;
	uint32_t tmr_ack = 0;
	uint8_t atTotal = 0;
	int8_t result = -1;
	bool dsOK = false;
	bool go = false;
#ifdef SET_GPS
	uint8_t sch = 0;
	int8_t cor_line = 6;
	uint32_t gps_tmr = get_tmr10(1);
#endif
	uint32_t cur_sec = get_tmr10(1);

	//-----   Основной цикл работы   -----

	while (!restart_flag) {

#ifdef SET_OLED_I2C
		//-----   блок обработки секундного события   -----
		if (check_tmr10(cur_sec)) {
			cur_sec = get_tmr10(_900ms);
			sec_to_string(scr);
			i2c_ssd1306_text_xy(mkLineCenter(scr, FONT_WIDTH), 1, 1, true);
			if (dsOK != sensPresent) {
				dsOK = sensPresent;
				strcpy(scr, "DS18B20 present");
				i2c_ssd1306_text_xy(mkLineCenter(scr, FONT_WIDTH), 1, temp_line, false);
			} else {
				if (temp != fTemp) {
					temp = fTemp;
#ifdef SET_FLOAT_PART
					s_float_t flo = {0, 0};
					floatPart(temp, &flo);
					sprintf(scr, "temp:%lu.%lu%c", flo.cel, flo.dro / 10000, 31);
#else
					sprintf(scr, "temp:%5.2f", temp);
#endif
					i2c_ssd1306_text_xy(mkLineCenter(scr, FONT_WIDTH), 1, temp_line, false);
				}
			}
		}
#endif
		//-------------------------------------------------------

		//-----   Блок чтения команд из очереди и отправки их GSM модулю   -----
		if (gsmToFlag) {//command to GSM module queue is ready
			if (getRECQ(buf, &gsmTo) >= 0) {
				if (HAL_UART_Transmit_DMA(portGSM, (uint8_t *)buf, strlen(buf)) != HAL_OK) devError |= devUART;
				else {
					if (strstr(buf, cmd_radio[fSCAN].cmd)) {
						gsmFlags.rlist = 1;
						indList = 0;
						memset((uint8_t *)&freqList, 0, MAX_FREQ_LIST * sizeof(uint16_t));
					}
				}
				if (prn_cmd) Report(NULL, false, "%s", buf);
#ifdef SET_OLED_I2C
				strcpy(scr, buf);
				i2c_ssd1306_clear_line(at_line);
				if (strlen(scr) > MAX_FONT_CHAR) scr[MAX_FONT_CHAR] = '\0';
				i2c_ssd1306_text_xy(scr, 1, at_line, false);
#endif
				if (!at_auto) {
					if ((uk_ack = findCmd(buf, &grp_cmd, &cur_cmd)) != NULL) {
						//Report(NULL, true, "Command finded : grp_cmd=%d cur_cmd=%d%s", grp_cmd, cur_cmd, eol);
					}
				}
			}
		}
		//-----------------------------------------------------------------------

		//-----   Блок чтения сообщения от модуля GSM   -----
		if (gsmFromFlag) {//event (ack) from GSM module queue is ready
			if (getRECQ(buf2, &gsmFrom) >= 0) {
#ifdef SET_OLED_I2C
				if (clin > elin) {
					clin = slin;
					i2c_ssd1306_clear_lines(clin, elin);
				}
				strncpy(scr, buf2, sizeof(scr) - 1);
				if (strlen(scr) > MAX_FONT_CHAR) scr[MAX_FONT_CHAR] = '\0';
				i2c_ssd1306_text_xy(scr, 1, clin++, false);
#endif
				Report(NULL, false, "%s", buf2);
				//
				if ((uks = strstr(buf2, "\r\n")) != NULL) *uks = '\0';
				result = parseEvent(buf2, (void *)&gsmFlags);// Анализ сообщения от модуля GSM
				//
				if (gsmFlags.begin) {//  Начать работу нитки с самого начала (по события 'SMS Ready')
					*(unsigned int *)&gsmFlags = 0;
					gsmFlags.sReady = 1;
					cur_cmd = grp_cmd = -1;
					cmd_err = CMD_REPEAT;
					uk_cmd = uk_ack = NULL;
					next_cmd = true;
					at_auto = false;
					tmr_next = tmr_ack = 0;
					atTotal = 0;
					go = false;
					gsmInit = sntpInit = false;
					netInit = false;
					HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_SET);

					Report(NULL, true, "Restart main loop !\r\n");
					i2c_ssd1306_clear_lines(at_line, elin);

					tmr_cmd = get_tmr(1);
					cur_sec = get_tmr10(1);
#ifdef SET_GPS
					gps_tmr = cur_sec;
#endif

					continue;
				}
				//-----   Анализ данных от стороннего tcp сервера   -----
				if (gsmFlags.connect) {
					if (gsmFlags.ack) {
						gsmFlags.ack = 0;
						parseACK(buf2);
					}
				}//------------------------------------------------------
				//-----   Блок отправки данных стороннему серверу через установленное tcp-соединения   -----
				if (gsmFlags.prompt) {//send json_data to external tcp server
					gsmFlags.prompt = 0;
					uint16_t dln = mkData(cmdBuf);
					if (dln) {
						if (HAL_UART_Transmit_DMA(portGSM, (uint8_t *)cmdBuf, dln) != HAL_OK) {
							devError |= devUART;
						} else {
							HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_SET);//off led
							gsmFlags.send = 0;
							Report(NULL, false, "%s", cmdBuf);
						}
					}
				}
				//-------------------------------------------------------------------------------------------
				// Для режима авто - частиный анализ полученных от GSM модуля данных (сообщений)
//				if (at_auto) {
					if (uk_ack) {
						if (strstr(buf2, uk_ack)) {
							if (at_auto) {
								next_cmd = true;
								tmr_next = get_tmr10(_200ms);
							}
							bool repeat = false;
							switch (cur_cmd) {
								case iCREG:
									if (grp_cmd == seqInit) {
										if (at_auto) {
											if (!gsmFlags.reg) repeat = true;
										}
									}
								break;
								case iCSQ:
									if (grp_cmd == seqInit) {
										if (at_auto) {
											if (gsmRSSI <= dBmRSSI[0]) repeat = true;
										}
									}
								break;
								case iCGATT:
									if (gsmFlags.ok) gsmFlags.cGat = 1;
								break;
								case iCMEE:
									if (gsmFlags.ok) gsmFlags.cmee = 1;
								break;
							}
							if (repeat) {
								next_cmd = false;
								tmr_ack = get_tmr10(0);
							}
						} else {
							if (gsmFlags.error & at_auto) {//example : +CME ERROR: 100
								next_cmd = false;
								tmr_next = 0;
								tmr_ack = get_tmr10(0);
							} else {
								switch (grp_cmd) {
									case seqInit:
										if (cur_cmd == iGSN) {
											if (strlen(buf2) >= 15) memcpy(gsmIMEI, buf2, 15);
										}
									break;
								}
							}
						}
					}
//				}
				//-----------------------------------------------------------------------------------
			}
		}
		//--- Начальная активация таймера периодической выдачи данных стороннему серверу ---
		if (!tmr_send) {// && !at_auto) {
			if (gsmFlags.connect) tmr_send = get_tmr10(0);
		}
		//--- Блок выдачи команды отправки данных на сторонний сервер ---
		if (tmr_send && !gsmFlags.send) {
			if (check_tmr10(tmr_send)) {
				if (gsmFlags.connect) {
					//
					//  cmd_send start
					if (cmds) { free(cmds); cmds = NULL; }
					cmds = (char *)calloc(1, CMD_LEN + 1);
					if (cmds) {
						strcpy(cmds, cmd_any[cCIPSEND].cmd);
						if (putRECQ(cmds, &gsmTo) < 0) {
							devError |= devQue;
							free(cmds);
							cmds = NULL;
						} else prn_cmd = true;
					} else devError |= devMem;
					gsmFlags.busy = 1;
					//
					//
					tmr_send = get_tmr10(send_period);// данные на сервер будем отправлять с периодом = send_period (10 сек.)
				} else tmr_send = 0;
			}
		}
		//------------------------------------------------------------

		//   Блок анализа флагой локальный команд и запуска соответствующей цепочки AT-команд
		if (!at_auto) {
			go = false;
			if (ini_flag) {// Запускаем цепочку команд инициализации молуля GSM
				ini_flag = 0;
				gsmInit = 0;
				grp_cmd = seqInit;
				atTotal = cmd_iniMax;
				cmd_adr = &cmd_ini[0].cmd[0];
				go = true;
			} else if (sntp_flag) {// Запускаем цепочку команд получения даты/времени от sntp сервера
				sntp_flag = 0;
				sntpInit = 0;
				grp_cmd = seqTime;
				atTotal = cmd_timeMax;
				cmd_adr = &cmd_time[0].cmd[0];
				go = true;
			} else if (radio_flag) {// Запускаем цепочку команд активации режима radio у модуля GSM
				radio_flag = 0;
				grp_cmd = seqRadio;
				atTotal = cmd_radioMax;
				cmd_adr = &cmd_radio[0].cmd[0];
				go = true;
			}  else if (net_flag) {// Запускаем цепочку команд для установления соединения со сторонним сервером
				net_flag = false;
				grp_cmd = seqNet;
				atTotal = cmd_netMax;
				cmd_adr = &cmd_net[0].cmd[0];
				go = true;
			}
			if (go) {
				gsmFlags.busy = 1;
				cmd_err = CMD_REPEAT;
				tmr_cmd = tmr_ack = 0;
				at_auto = next_cmd = true;
				tmr_next = get_tmr10(_1s);
			}
		}
		//   Блок запуска соответствующей цепочки AT-команд в режиме 'авто'
		if (tmr_cmd) {//start send at_commands in mode auto
			if (check_tmr(tmr_cmd)) {
				if (gsmFlags.sReady) {
					if (!gsmInit) {
						grp_cmd = seqInit;
						atTotal = cmd_iniMax;
						cmd_adr = &cmd_ini[0].cmd[0];
					} else if (!sntpInit) {
						grp_cmd = seqTime;
						atTotal = cmd_timeMax;
						cmd_adr = &cmd_time[0].cmd[0];
					} else if (!netInit) {
						grp_cmd = seqNet;
						atTotal = cmd_netMax;
						cmd_adr = &cmd_net[0].cmd[0];
					}
					gsmFlags.busy = 1;
					tmr_cmd = tmr_ack = 0;
					at_auto = true;
					next_cmd = true;
					tmr_next = get_tmr10(_1s);
				}
			}
		}
		//
		// Для режима 'авто' - блок последовательной выдачи команд модулю GSM
		if (at_auto) {
			if (tmr_next) {
				if (check_tmr10(tmr_next)) {
					tmr_next = 0;
					if (next_cmd) {
						next_cmd = false;
						cur_cmd++;
						gsmFlags.busy = 1;
						switch (grp_cmd) {
							case seqInit:// группа команд инициализации
							case seqTime:// группа команд получения дата/время от sntp сервера
							case seqRadio:// группа команд активации режима radio у модуля GSM
							case seqNet:// группа команд установления соединения со сторонним сервером
								if (cur_cmd < atTotal) {
									int lens = CMD_LEN;
									uk_cmd = cmd_adr + (cur_cmd * sizeof(ats_t));//cmd
									uk_ack = uk_cmd + CMD_LEN;//cmd_adr[cur_cmd]->ack;
									switch (grp_cmd) {
										case seqTime:
											if (cur_cmd == tSAPBR31) {//{"AT+SAPBR=3,1,\"APN\",","OK"},//\"internet\"\r\n" | "APN" + eol
												lens = sprintf(cmdBuf, "%s\"%s\"%s", uk_cmd, APN, eol);
												uk_cmd = &cmdBuf[0];
											} else if (cur_cmd == tCNTP_SRV) {//{"AT+CNTP=","OK"},//\"pool.ntp.org\",8\r\n" | "SNTP",TZONE<<2 + eol
												strncpy(cntpSRV, SNTP, sizeof(cntpSRV) - 1);
												lens = sprintf(cmdBuf, "%s\"%s\",%u%s", uk_cmd, SNTP, (TZONE << 2), eol);
												uk_cmd = &cmdBuf[0];
											} else if (cur_cmd == tCCLK) gsmFlags.okDT = 0;
										break;
										case seqRadio:
											if (cur_cmd == fOPEN) {
												gsmFlags.ropen = 1;
												indList = 0;
											} else if (cur_cmd == fCLOSE) {
												gsmFlags.ropen = 0;
											} else if (cur_cmd == fFREQ) {//{"AT+FMFREQ=","OK"}//,//1025 ; установить чатоту 102.5 Мгц | 1025 + eol
												uint16_t fr = 1025;
												while (indList < MAX_FREQ_LIST) {
													if (freqList[indList]) {
														fr = freqList[indList];
														break;
													}
													indList++;
												}
												lens = sprintf(cmdBuf, "%s%u%s", uk_cmd, fr, eol);
												uk_cmd = &cmdBuf[0];
												gsmFlags.play = 1;
											}
										break;
										case seqNet:
											if (cur_cmd == nCSTT) {//"AT+CSTT=\"internet\",\"beeline\",\"beeline\"\r\n","OK", | "APN","LOGIN","PASSWORD" + eol
												lens = sprintf(cmdBuf, "%s\"%s\",\"%s\",\"%s\"%s", uk_cmd, APN, LOGIN, PASSWORD, eol);
												uk_cmd = &cmdBuf[0];
											} else if (cur_cmd == nCIPSTART) {//"AT+CIPSTART=\"TCP\",","CONNECT OK"//\"213.149.17.142\",8778\r\n","CONNECT OK"//after OK -> CONNECT OK | "SRV"
												lens = sprintf(cmdBuf, "%s\"%s\",%u%s", uk_cmd, SRV_ADR, SRV_PORT, eol);
												cmdBuf[lens] = '\0';
												uk_cmd = &cmdBuf[0];
											}
										break;
									}
									/**/
									if (cmds) { free(cmds); cmds = NULL; }
									cmds = (char *)calloc(1, lens + 1);
									if (cmds) {
										strcpy(cmds, uk_cmd);
										if (putRECQ(cmds, &gsmTo) < 0) {
											devError |= devQue;
											free(cmds);
											cmds = NULL;
										} else {
											tmr_ack = get_tmr10(_15s);//wait answer from module 10 sec
										}
									} else devError |= devMem;
									//
									if (devError) {
										cmd_err = CMD_REPEAT;
										cur_cmd = -1;
										next_cmd = at_auto = false;
										uk_cmd = uk_ack = NULL;
										tmr_next = tmr_ack = 0;
									}
									prn_cmd = true;
								} else {
									cmd_err = CMD_REPEAT;
									cur_cmd = -1;
									next_cmd = at_auto = false;
									uk_cmd = uk_ack = NULL;
									tmr_next = tmr_ack = 0;
									gsmFlags.busy = 0;
									if (grp_cmd == seqInit) {
										gsmInit = true;
										if (!sntpInit) {
											grp_cmd = seqTime;
											tmr_cmd = get_tmr(1);
											//gsmFlags.busy = 1;
										}
									} else if (grp_cmd == seqTime) {
										sntpInit = true;
										if (!netInit) {
											grp_cmd = seqNet;
											tmr_cmd = get_tmr(1);
											//gsmFlags.busy = 1;
										}
									} else if (grp_cmd == seqNet) {
										netInit = true;
										//gsmFlags.busy = 1;
									}
								}
							break;
						}//switch (grp_cmd)
					}//if (next_cmd)
				}//if (check_tmr10(tmr_next))
			}//if (tmr_next)

			// Блок обработки события - 'нет ответа от модуля GSM' - timeout
			if (tmr_ack) {
				if (check_tmr10(tmr_ack)) {
					if (cmd_err) cmd_err--;
					if (!cmd_err) {// stop at_commands sequence
						cmd_err = CMD_REPEAT;
						tmr_next = get_tmr10(_750ms);
					} else {
						if (grp_cmd != seqNet) {
							if (cur_cmd > 0) cur_cmd--;
							tmr_next = get_tmr10(_2s);
						} else {
							cur_cmd = atTotal = cmd_timeMax;
							cmd_err = CMD_REPEAT;
							tmr_next = get_tmr10(_750ms);
						}
					}
					next_cmd = true;
					tmr_ack = 0;
				}
			}
			//---------------------------------------------------------------
		} else {
			// если режим 'авто' выдачи команд завершен - проверяем нет ли необходимости
			// выполнить одиночную команду из группы 'any'
			//
			if (result != -1) {//== cCIPSHUT
				if (cmds) { free(cmds); cmds = NULL; }
				cmds = (char *)calloc(1, CMD_LEN + 1);
				if (cmds) {
					strcpy(cmds, cmd_any[result].cmd);
					if (putRECQ(cmds, &gsmTo) < 0) {
						devError |= devQue;
						free(cmds);
						cmds = NULL;
					} else prn_cmd = true;//Report(NULL, false, cmds);
				} else devError |= devMem;
				result = -1;
			}
			//----------------------------------------------------------------------------
		}
		//
#ifdef SET_SMS
		//--- блок конкатенации частей смс по истечении таймера ожидания прихода всех частей ---
		if (wait_sms) {
			if (check_tmr(wait_sms)) {
				wait_sms = 0;
				sms_total = getSMSTotalCounter();
				if (sms_total) {
					*SMS_text = '\0';   sms_num = 0;
					if (ConcatSMS(SMS_text, sms_total, &sms_num, &sms_len) == sms_total) {
						Report(NULL, true, "[SMS] Concat message #%u with len %u by timeout:\r\n%.*s\r\n",
								     sms_num, sms_len, sms_len, SMS_text);
#ifdef SET_SMS_QUEUE
						//----------------  check : command present in sms ?   ------------------------
						checkSMS(SMS_text, fromNum);
						//-----------------------------------------------------------------------------
						//
						*smsTMP = '\0';
						if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsTMP, sizeof(smsTMP) - 1)) {
							if (addSRECQ(smsTMP, &smsq) < 0) {
								Report(NULL, true, smsTMP);
							}
						}
#endif
					}
				}
				InitSMSList();
			}
		}
		//--------------------------------------------------------------------------------
#endif
		//
		//
		//
		if (prn_flags) {// Событие - установлен флаг печати структуры состояния (все рабочие флаги)
			prn_flags = false;
			prnFlags((void *)&gsmFlags);
		} else if (prn_rlist) {// Событие - печать лист с чатотами просканированых ФМ-радиостанций
			prn_rlist = false;
			prnRList();
		} else if (prn_freemem) {// Событие - печатать количество сободной динамической памяти из 'кучи'
			prn_freemem = false;
			Report(__func__, true, "Free heap memory size %lu\r\n", xPortGetFreeHeapSize());
		} else if (clr_flag) {
			clr_flag = false;
			devError = 0;
			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//LED OFF
			i2c_ssd1306_clear_line(err_line);
			Report(__func__, true, "Clear all errors (devError)%s", eol);
		}

		if (devError) {//  Блок выдачи на дисплей ошибок на устройстве, если таковые есть
#ifdef SET_OLED_I2C
			errNameStr(devError, scr);
			if (strlen(scr) > MAX_FONT_CHAR) scr[MAX_FONT_CHAR] = '\0';
			i2c_ssd1306_text_xy(mkLineCenter(scr, FONT_WIDTH), 1, err_line, true);
#endif
			errLedOn(NULL);
			devError = 0;
		}
		//-------------------------------------------------------------------------------

		//   Блок чтения данных от GPS модуля из очереди сообщений (~ каждую секунду)
#ifdef SET_GPS
		if (gpsFromFlag) {//queue for messages from GPS module is ready
			if (check_tmr10(gps_tmr)) {
				gps_tmr = get_tmr10(_50ms);
				if (getRECQ(buf, &gpsFrom) >= 0) {//get message from queue if present
	#ifdef SET_GPS_DEBUG
					Report(NULL, true, "%s%s", buf, eol);
	#endif
					if (gpsParse(buf)) {// анализ и преобразование данных из NMEA-формата в структуру с валидными данными
						sch++;
						if (sch == MAX_NMEA_MSG) {
							sch = 0;
							if (prnGpsFlag) Report(NULL, true, "%s%s", gpsPrint(buf), eol);
							// Вывод данных на OLED дисплей
							s_float_t flo = {0,0};
							floatPart(GPS.dec_latitude, &flo);  sprintf(scr,            "%02lu.%04lu ",  flo.cel, flo.dro / 100);
							floatPart(GPS.dec_longitude,&flo);  sprintf(scr+strlen(scr),"%02lu.%04lu", flo.cel, flo.dro / 100);
							floatPart(GPS.msl_altitude,  &flo); sprintf(scr+strlen(scr),"\n sat:%d alt:%lu",GPS.satelites, flo.cel);
							i2c_ssd1306_text_xy(scr, 1, cor_line, false);
							gps_tmr = get_tmr10(_700ms);
						}
					}//-----------------------------------------------------------------------------------------------------
				}
			}
		}
#endif
		//
#ifdef SET_W25FLASH
		//   Блок работы с внешней flash памятью. Доступные функции : чтение, запись и стирание
		if (flag_sector) {
			flag_sector = false;
			switch (cmd_sector) {
				case sRead:
				case sNext:
				{
					uint32_t w25_adr = (adr_sector * W25qxx_getSectorSize()) + offset_sector;
					uint32_t dlin = list_sector;
					int step = 32;
					uint32_t ind = 0;
					W25qxx_ReadSector(fs_work, adr_sector, offset_sector, dlin);
					Report(NULL, false, "Read sector:%d offset:%d len:%u%s", adr_sector, offset_sector, dlin, eol);
					while (ind < dlin) {
						strf[0] = '\0';
						while (1) {
							sprintf(strf+strlen(strf), "%06X ", (unsigned int)w25_adr);
							for (int i = 0; i < step; i++) sprintf(strf+strlen(strf), " %02X", fs_work[i + ind]);
							strcat(strf, eol);
							w25_adr += step;
							ind += step;
							if (!(ind % W25qxx_getPageSize())) break;
						}
						Report(NULL, false, "%s", strf);
					}
				}
				break;
				case sWrite:
				{
					uint32_t ss = W25qxx_getSectorSize();
					if (!W25qxx_IsEmptySector(adr_sector, 0, ss)) W25qxx_EraseSector(adr_sector);
					memset(fs_work, byte_write, ss);
					if (len_write != -1) ss = len_write;
					W25qxx_WriteSector(fs_work, adr_sector, offset_sector, ss);
					Report(NULL, false, "Fill sector:%d byte:%02X len:%d done%s", adr_sector, byte_write, ss, eol);
				}
				break;
				case sErase:
					W25qxx_EraseSector(adr_sector);
					Report(NULL, false, "Erase sector:%d done%s", adr_sector, eol);
				break;
			}
			last_cmd_sector =  cmd_sector;
			cmd_sector = sNone;
		}
		//
#ifdef SET_FAT_FS
		if (cat_flag) {
			cat_flag = 0;
			rdFile(cfg);
		}
#endif
		//
#endif
		//
		osDelay(10);
		//
	}
	//
	Report(NULL, true, "Restart...\n");
	HAL_Delay(1000);
	NVIC_SystemReset();// Рестарт контроллера по локальной команде 'restart'
	//
	//
	//
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTemp */
/**
* @brief Function implementing the tempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemp */
void StartTemp(void *argument)
{
  /* USER CODE BEGIN StartTemp */

	//---------------------------------------------------------------------
	//
	//  Нитка чтения и преобразования данных с датчика температуры DS18B20
	//
	//---------------------------------------------------------------------

#ifdef SET_TEMP_SENSOR
  /* Infinite loop */

	Report(__func__, true, "Start sensor thread...(%lu)\r\n", xPortGetFreeHeapSize());

	uint8_t	Ds18b20TryToFind = 5;

	//  Блок определения есть ли наш датчик на линии DS18B20_Pin (5 попыток)
	do
	{
		OneWire_Init(&OneWire, DS18B20_GPIO_Port, DS18B20_Pin);
		TempSensorCount = 0;

		while (HAL_GetTick() < 3000) Ds18b20Delay(50);

		OneWireDevices = OneWire_First(&OneWire);
		while (OneWireDevices) {
			Ds18b20Delay(100);
			TempSensorCount++;
			OneWire_GetFullROM(&OneWire, ds18b20[TempSensorCount - 1].Address);
			OneWireDevices = OneWire_Next(&OneWire);
		}
		if (TempSensorCount > 0) break;
		Ds18b20TryToFind--;

		if (restart_flag) break;

	} while (Ds18b20TryToFind > 0);
	//--------------------------------------------------------------


	if (TempSensorCount > 0) sensPresent = true;//  Датчик обнаружен !!!


	//  Цикл чтения данных и преобразования значения температуры в градусы Цельсия
	while (!restart_flag && sensPresent) {//Ds18b20TryToFind)

		for (uint8_t i = 0; i < TempSensorCount; i++) {
			osDelay(50);
			DS18B20_SetResolution(&OneWire, ds18b20[i].Address, DS18B20_Resolution_12bits);
			osDelay(50);
			DS18B20_DisableAlarmTemperature(&OneWire,  ds18b20[i].Address);
		}
		for (;;) {
			while (!_DS18B20_UPDATE_INTERVAL_MS) {
				if (Ds18b20StartConvert == 1) break;
				osDelay(10);
			}
			Ds18b20Timeout=_DS18B20_CONVERT_TIMEOUT_MS/10;
			DS18B20_StartAll(&OneWire);
			osDelay(100);
			while (!DS18B20_AllDone(&OneWire)) {
				osDelay(10);
				Ds18b20Timeout -= 1;
				if (!Ds18b20Timeout) break;
			}
			if (Ds18b20Timeout > 0) {
				for (uint8_t i = 0; i < TempSensorCount; i++) {
					osDelay(1000);
					ds18b20[i].DataIsValid = DS18B20_Read(&OneWire, ds18b20[i].Address, &ds18b20[i].Temperature);
				}
			} else {
				for (uint8_t i = 0; i < TempSensorCount; i++) ds18b20[i].DataIsValid = false;
			}
			//
			fTemp = ds18b20[0].Temperature;// Глобальная переменная для хранения значения температуры !!!
			//
			Ds18b20StartConvert = 0;
			osDelay(2000);//_DS18B20_UPDATE_INTERVAL_MS);
			//
		}

	}

#endif


  	osThreadExit();

  /* USER CODE END StartTemp */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
//------------------------------------------------------
//
//   CallBack функция таймера
//
//------------------------------------------------------
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM3) {// период срабатывания 10 мсек.
	  HalfSecCounter++;//+10ms
	  if (!(HalfSecCounter % _1s)) {//seconda
		  inc_secCounter();//secCounter++;
		  if (screenON) {
			  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);//set ON/OFF LED1
		  } else {
			  if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_SET)
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  	  }
	  }
  }
#ifdef SET_TEMP_SENSOR
  else if (htim->Instance == TIM10) {// период срабатывания 1 мксек.
	  //
  }
#endif
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
