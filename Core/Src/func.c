
#include "hdr.h"


#include "main.h"
#include "func.h"


//******************************************************************************************

#ifdef USED_FREERTOS
	uint32_t waitRTC = 200;
#endif

//-----------------------------------------------------------------------------
#ifdef SET_FLOAT_PART
void floatPart(float val, s_float_t *part)
{
	part->cel = (uint32_t)val;
	part->dro = (val - part->cel) * 1000000;
}
#endif
//-----------------------------------------------------------------------------
//          Функции формирования временных интервалов,
//           а также контроля за этими интервалами
//
uint32_t get_secCounter()
{
	return secCounter;
}
//----------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//----------------------------------------------
uint32_t get_hsCounter()
{
	return HalfSecCounter;
}
//----------------------------------------------
void inc_hsCounter()
{
	HalfSecCounter++;
}
//----------------------------------------------
uint32_t get_tmr10(uint32_t ms)
{
	return (get_hsCounter() + ms);
}
//----------------------------------------------
bool check_tmr10(uint32_t ms)
{
	return (get_hsCounter() >= ms ? true : false);
}
//----------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//----------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//----------------------------------------------
uint64_t get_hstmr(uint64_t hs)
{
	return (get_hsCounter() + hs);
}
//----------------------------------------------
bool check_hstmr(uint64_t hs)
{
	return (get_hsCounter() >= hs ? true : false);
}
//-----------------------------------------------------------------------------------------
void gsmReset()
{
	/*HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);//LED ON
	HAL_Delay(750);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);//LED OF*/
}
//-----------------------------------------------------------------------------------------

#ifdef SET_SMS
	#ifdef SET_SMS_QUEUE
bool initSRECQ(s_recq_t *q)//s_recq_t recq;
{
//#ifdef USED_FREERTOS
//	if (osSemaphoreAcquire(smsSem, 250) == osOK) {
//#endif
		q->put = q->get = 0;
		for (uint8_t i = 0; i < MAX_SQREC; i++) {
			q->rec[i].id = i;
#ifdef SET_RECQ_STATIC
			q->rec[i].adr[0] = '\0';
#else
			q->rec[i].adr = NULL;
#endif
		}
//#ifdef USED_FREERTOS
//		osSemaphoreRelease(smsSem);
//	}
//#endif
	return true;
}
//-----------------------------------------------------------------------------
void clearSRECQ(s_recq_t *q)
{
#ifdef USED_FREERTOS
	if (osSemaphoreAcquire(smsSem, 250) == osOK) {
#endif
		q->put = q->get = 0;
		for (uint8_t i = 0; i < MAX_SQREC; i++) {
			q->rec[i].id = i;
#ifdef SET_RECQ_STATIC
			q->rec[i].adr[0] = '\0';
#else
			free(q->rec[i].adr);
			q->rec[i].adr = NULL;
#endif
		}
#ifdef USED_FREERTOS
		osSemaphoreRelease(smsSem);
	}
#endif
}
//-----------------------------------------------------------------------------
int8_t putSRECQ(char *adr, s_recq_t *q)
{
int8_t ret = -1;
#ifdef USED_FREERTOS
	if (osSemaphoreAcquire(smsSem, 250) == osOK) {
#endif
#ifdef SET_RECQ_STATIC
		if (!strlen(q->rec[q->put].adr)) {
			int len = strlen(adr);
			if (len >= REC_BUF_LEN) len = REC_BUF_LEN - 1;
			memcpy((char *)&q->rec[q->put].adr[0], adr, len);
			q->rec[q->put].adr[len] = '\0';
#else
		if (q->rec[q->put].adr == NULL) {
			q->rec[q->put].adr = adr;
#endif
			ret = q->rec[q->put].id;
			q->put++;   if (q->put >= MAX_SQREC) q->put = 0;
		}
#ifdef USED_FREERTOS
		osSemaphoreRelease(smsSem);
	}
#endif

	return ret;
}
//-----------------------------------------------------------------------------
int8_t getSRECQ(char *dat, s_recq_t *q)
{
int8_t ret = -1;
int len = 0;

#ifdef USED_FREERTOS
	if (osSemaphoreAcquire(smsSem, 250) == osOK) {
#endif
	#ifdef SET_RECQ_STATIC
			len = strlen(q->rec[q->get].adr);
			if (len) {
				ret = q->rec[q->get].id;
				memcpy(dat, q->rec[q->get].adr, len);
				q->rec[q->get].adr[0] = '\0';
			}
	#else
			if (q->rec[q->get].adr != NULL) {
				len = strlen(q->rec[q->get].adr);
				ret = q->rec[q->get].id;
				memcpy(dat, q->rec[q->get].adr, len);
				free(q->rec[q->get].adr);
				q->rec[q->get].adr = NULL;
			}
	#endif

			if (ret >= 0) {
				*(dat + len) = '\0';
				q->get++;   if (q->get >= MAX_SQREC) q->get = 0;
			}
#ifdef USED_FREERTOS
			osSemaphoreRelease(smsSem);
		}
#endif

		return ret;
}
//------------------------------------------------------------------------------------------
int8_t addSRECQ(char *txt, s_recq_t *q)
{
int8_t nrec = -1;
uint16_t txt_len = strlen(txt);

#ifdef SET_RECQ_STATIC
	if (txt_len >= REC_BUF_LEN) {
		txt_len = REC_BUF_LEN - 1;
		txt[txt_len] = '\0';
	}
	if ((nrec = putSRECQ(txt, q)) >= 0) {
		Report(__func__, true, "put record to queue OK (id=%d len=%d)\r\n", nrec, txt_len);
	} else {
		Report(__func__, true, "put record to queue error (len=%d)\r\n", txt_len);
	}
#else
	int need_len = txt_len + 1;
	if (need_len <= MAX_UART_BUF) need_len = MAX_UART_BUF;
	char *rc = (char *)calloc(1, (size_t)need_len);
	if (rc) {
		int got_len = strlen(rc);
		if (got_len >= need_len) {
			memcpy(rc, txt, txt_len);
			*(rc + txt_len) = '\0';
			if ((nrec = putSRECQ(rc, q)) >= 0) {
				Report(__func__, true, "put record to queue OK (id=%d len=%d/%d)\r\n", nrec, need_len, got_len);
		    } else {
				Report(__func__, true, "put record to queue error (len=%d/%d)\r\n", need_len, got_len);
				free(rc);
			}
		} else {
			Report(__func__, true, "error memory size %d != %d\r\n", need_len, got_len);
			free(rc);
		}
	} else devError |= devMem;//Report(true, "[%s] : error get memory (len=%d)\r\n", __func__, need_len);
#endif

	return nrec;
}
	#endif
#endif

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//        Функция инициализации очереди сообщений
//
bool initRECQ(s_recq_t *q)//s_recq_t recq;
{
	//if (osSemaphoreAcquire(mainBinSemHandle, 100) == osOK) {

		q->put = q->get = 0;
		q->lock = 0;
		for (uint8_t i = 0; i < MAX_QREC; i++) {
			q->rec[i].id = i;
			q->rec[i].adr = NULL;
		}

		//osSemaphoreRelease(mainBinSemHandle);
	//}

	return true;
}
//-----------------------------------------------------------------------------
//                     Очистка очереди
//
bool clearRECQ(s_recq_t *q)
{
	//if (osSemaphoreAcquire(mainBinSemHandle, 100) == osOK) {

	while (q->lock) {}
	q->lock = 1;

		q->put = q->get = 0;
		for (uint8_t i = 0; i < MAX_QREC; i++) {
			q->rec[i].id = i;
			free(q->rec[i].adr);//freeMem(q->rec[i].adr);
			q->rec[i].adr = NULL;
		}

		//osSemaphoreRelease(mainBinSemHandle);
	//}

	q->lock = 0;

	return false;
}
//-----------------------------------------------------------------------------
//                 Функция добавляет сообщение в очередь
//
int8_t putRECQ(char *adr, s_recq_t *q)
//int8_t putRECQ(uint16_t dat, s_recq_t *q)
{
int8_t ret = -1;

	//if (osSemaphoreAcquire(mainBinSemHandle, 10) == osOK) {

	while (q->lock) {}
	q->lock = 1;

		if (q->rec[q->put].adr == NULL) {
			q->rec[q->put].adr = adr;
			ret = q->rec[q->put].id;
			q->put++;   if (q->put >= MAX_QREC) q->put = 0;
		}

		//osSemaphoreRelease(mainBinSemHandle);
	//}

	q->lock = 0;

	return ret;
}
//-----------------------------------------------------------------------------
//            Функция извлекает сообщение из очереди
//
int8_t getRECQ(char *dat, s_recq_t *q)
//int8_t getRECQ(uint16_t *dat, s_recq_t *q)
{
int8_t ret = -1;
int len = 0;

	//if (osSemaphoreAcquire(mainBinSemHandle, 0) == osOK) {

	while (q->lock) {}
	q->lock = 1;

		if (q->rec[q->get].adr != NULL) {
			len = strlen(q->rec[q->get].adr);
			ret = q->rec[q->get].id;
			memcpy(dat, q->rec[q->get].adr, len);
			free(q->rec[q->get].adr);//freeMem(q->rec[q->get].adr);
			q->rec[q->get].adr = NULL;
		}

		if (ret >= 0) {
			*(dat + len) = '\0';
			q->get++;   if (q->get >= MAX_QREC) q->get = 0;
		}

		//osSemaphoreRelease(mainBinSemHandle);

	//}

	q->lock = 0;

	return ret;
}
//-----------------------------------------------------------------------------------------
// set LED_ERROR when error on and send message to UART1 (in from != NULL)
//     from - name of function where error location
void errLedOn(const char *from)
{
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//LED ON
	HAL_Delay(25);
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);//LED OFF

	if (from) Report(NULL, true, "Error in function '%s'\r\n", from);
}
//------------------------------------------------------------------------------------------
//        Функция устанавливает время (в формате epochtime) в модуле RTC контроллера
//
void set_Date(time_t ep)
{
bool yes = false;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;
bool os_core = false;

	gmtime_r(&ep, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	sTime.Hours   = ts.tm_hour + tZone;
	sTime.Minutes = ts.tm_min;
	sTime.Seconds = ts.tm_sec;

	//__HAL_RCC_RTC_DISABLE();
	if (coreStatus == osOK) os_core = true;

#ifdef USED_FREERTOS
	if (os_core) {
		if (osMutexAcquire(rtcMutexHandle, waitRTC) == osOK) {
	}
#endif
		if (HAL_RTC_SetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
		else {
			if (HAL_RTC_SetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
			else {
				//if (devError & devRTC) devError &= ~devRTC;
				yes = true;
			}
		}

#ifdef USED_FREERTOS
	if (os_core) {
			osMutexRelease(rtcMutexHandle);
		}
	}
#endif

	if (yes) setDate= true;

	//__HAL_RCC_RTC_ENABLE();
}
//------------------------------------------------------------------------------------------
//       Функция возвращает текущее время (в формате epochtime) из модуля RTC
//
uint32_t get_Date()
{
	if (!setDate) return get_tmr(0);

	struct tm ts;

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
#ifdef USED_FREERTOS
	if (osMutexAcquire(rtcMutexHandle, waitRTC) == osOK) {
#endif
		if (HAL_RTC_GetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
		ts.tm_hour = sTime.Hours;
		ts.tm_min  = sTime.Minutes;
		ts.tm_sec  = sTime.Seconds;

		if (HAL_RTC_GetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
		ts.tm_wday = sDate.WeekDay;
		ts.tm_mon  = sDate.Month - 1;
		ts.tm_mday = sDate.Date;
		ts.tm_year = sDate.Year;
#ifdef USED_FREERTOS
		osMutexRelease(rtcMutexHandle);
	}
#endif

	return ((uint32_t)mktime(&ts));
}
//------------------------------------------------------------------------------------
//   Функция формирует символьную строку с датой и временем из значения epochtime
//         и возвращает длинну сформированной символьной строки
//
int sec_to_str_time(uint32_t sec, char *stx)
{
int ret = 0;
bool yes = false;

	if (!setDate) {//no valid date in RTC
		uint32_t day = sec / (60 * 60 * 24);
		sec %= (60 * 60 * 24);
		uint32_t hour = sec / (60 * 60);
		sec %= (60 * 60);
		uint32_t min = sec / (60);
		sec %= 60;
		ret = sprintf(stx, "%lu.%02lu:%02lu:%02lu", day, hour, min, sec);
	} else {//in RTC valid date (epoch time)
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
#ifdef USED_FREERTOS
		if (osMutexAcquire(rtcMutexHandle, waitRTC) == osOK) {
#endif
			if (HAL_RTC_GetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
			else {
				if (HAL_RTC_GetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;////errLedOn(__func__);
				else {
					yes = true;
				}
			}
#ifdef USED_FREERTOS
			osMutexRelease(rtcMutexHandle);
		}
#endif
		if (yes) ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u",
			   	   	   	   	   	   sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, sTime.Seconds);
	}

	return ret;
}
//----------------------------------------------------------------------------------------
//   Функция формирует символьную строку с датой и временем из значения epochtime
//         и возвращает длинну сформированной символьной строки
//
int sec_to_string(uint32_t sec, char *stx)
{
int ret = 0;
bool yes = false;

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
#ifdef USED_FREERTOS
	if (osMutexAcquire(rtcMutexHandle, waitRTC) == osOK) {
#endif
		if (HAL_RTC_GetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			if (HAL_RTC_GetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
			else {
				//if (devError & devRTC) devError &= ~devRTC;
				yes = true;
			}
		}
#ifdef USED_FREERTOS
		osMutexRelease(rtcMutexHandle);
	}
#endif
	if (yes) ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u ",
								sDate.Date, sDate.Month, sTime.Hours, sTime.Minutes, sTime.Seconds);

    return ret;
}
//------------------------------------------------------------------------------------------
//                Функция вывода символьной строки в порт логов (portLOG)
//
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...)
{
va_list args;
size_t len = MAX_UART_BUF;
int dl = 0;


	/*if (!uartRdy) {
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);//ON err_led
		return 1;
	} else {
		HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);//OFF err_led
	}*/

#ifdef SET_STATIC_MEM
	char *buff = &PrnBuf[0];
	buff[0] = 0;
#else
	char *buff = (char *)getMem(len);
	if (buff) {
#endif
		if (addTime) {
			uint32_t ep;
			if (!setDate) ep = get_secCounter();
					 else ep = extDate;
			dl = sec_to_string(ep, buff);
		}
		if (tag) dl += sprintf(buff+strlen(buff), "[%s] ", tag);
		va_start(args, fmt);
		vsnprintf(buff + dl, len - dl, fmt, args);
		uartRdy = 0;
		//er = HAL_UART_Transmit(&huart1, (uint8_t *)buff, strlen(buff), 1000);
		if (HAL_UART_Transmit_DMA(portLOG, (uint8_t *)buff, strlen(buff)) != HAL_OK)
			devError |= devUART;
		//else
		//	if (devError & devUART) devError &= ~devUART;
		/**/
		while (HAL_UART_GetState(portLOG) != HAL_UART_STATE_READY) {
			if (HAL_UART_GetState(portLOG) == HAL_UART_STATE_BUSY_RX) break;
			HAL_Delay(1);
		}
		/**/
		va_end(args);
#ifndef SET_STATIC_MEM
		freeMem(buff);
	}
#endif

	return 0;
}
//------------------------------------------------------------------------------------------
bool set_DT()
{
bool ret = false;
/*
struct tm ts = {
	.tm_hour = DT.hour,
	.tm_min  = DT.min,
	.tm_sec  = DT.sec,
	.tm_wday = 0,
	.tm_mon  = DT.mon - 1,
	.tm_mday = DT.day,
	.tm_year = DT.year
};

	time_t ep = mktime(&ts);
	if ((uint32_t)ep >= (uint32_t)epoch) {
		tZone = 0;
		set_Date(ep);
		tZone = DT.tz;
		ret = true;
	}
*/
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

	sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
	sDate.Month   = DT.mon;
	sDate.Date    = DT.day;
	sDate.Year    = DT.year;
	sTime.Hours   = DT.hour;// + tZone;
	sTime.Minutes = DT.min;
	sTime.Seconds = DT.sec;
#ifdef USED_FREERTOS
	if (osMutexAcquire(rtcMutexHandle, waitRTC) == osOK) {
#endif
		if (HAL_RTC_SetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
		else {
			if (HAL_RTC_SetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
			else {
				//if (devError & devRTC) devError &= ~devRTC;
				ret = true;
			}
		}
#ifdef USED_FREERTOS
		osMutexRelease(rtcMutexHandle);
	}
#endif

	if (ret) setDate = true;

	return ret;
}
//------------------------------------------------------------------------------------------
void prnFlags(void *g)
{
	gsmFlags_t *gf = (gsmFlags_t *)g;

	Report(NULL,
		   true,
		   "Flags:\n\trdy:%u\n\tcFun:%u\n\tcPin:%u\n\tCallReady:%u\n\tSMSReady:%u\n\tbegin:%u\n\treg:%u\n\tcGat:%u\n\tcmee:%u\n"
		   "\tcntp:%u\n\tokDT:%u\n\tstate:%s\n\tconnect:%u\n\tfail:%u\n\tclosed:%u\n\tshut:%u\n\tbusy:%u\n\terror:%u\n\tok:%u\n"
		   "\tsntpSRV:'%s'\n\tsntpDT:'%s'\n\timei:%s\n\tVcc:%u mv\r\n",
		   gf->rdy, gf->cFun, gf->cPin, gf->cReady, gf->sReady, gf->begin, gf->reg, gf->cGat, gf->cmee,
		   gf->tReady, gf->okDT, gsmState[gf->state], gf->connect,
		   gf->fail, gf->closed, gf->shut, gf->busy, gf->error, gf->ok,
		   cntpSRV, sntpDT, gsmIMEI, VCC);
}
//------------------------------------------------------------------------------------------
void prnRList()
{
char tp[(MAX_FREQ_LIST * 8) + 1] = {0};
int8_t i = -1;
s_float_t flo = {0,0};
float freq = 0.0;

	while (++i < MAX_FREQ_LIST) {
		if (freqList[i]) {
			freq = freqList[i];
			floatPart(freq / 10, &flo);
			sprintf(tp+strlen(tp), "%lu.%01lu ", flo.cel, flo.dro / 100000);
		}
	}
	int dl = strlen(tp);

	if (dl) {
		tp[dl - 1] = '\0';
		Report(NULL, true, "Radio freq_list MHz:[%s]\r\n", tp);
	} else {
		Report(NULL, true, "Radio freq_list is empty\r\n");
	}
}
//------------------------------------------------------------------------------------------
bool checkDT(char *str)//21/11/01,12:49:31+02
{
bool ret = false;
//int8_t cnt = 0;

	if (strchr(str, '+')) {
		if (sscanf(str, "%02d/%02d/%02d,%02d:%02d:%02d+%02d", &DT.year, &DT.mon, &DT.day, &DT.hour, &DT.min, &DT.sec, &DT.tz) == 7) ret = true;
	} else if (strchr(str, '-')) {
		if (sscanf(str, "%02d/%02d/%02d,%02d:%02d:%02d-%02d", &DT.year, &DT.mon, &DT.day, &DT.hour, &DT.min, &DT.sec, &DT.tz) == 7) ret = true;
	}
/*
	if (ret) {
		if (!IS_RTC_YEAR(DT.year)) {
			ret = false;
			cnt = 1;
		} else if (!IS_RTC_MONTH(DT.mon)) {
			ret = false;
			cnt = 2;
		} else if (!IS_RTC_DATE(DT.day)) {
			ret = false;
			cnt = 3;
		} else if (!IS_RTC_HOUR24(DT.hour)) {
			ret = false;
			cnt = 4;
		} else if (IS_RTC_MINUTES(DT.min)) {
			ret = false;
			cnt = 5;
		} else if (!IS_RTC_SECONDS(DT.sec)) {
			ret = false;
			cnt = 6;
		}
		if (!ret) {
			Report(__func__, false,
					"Error %d date/time %02d/%02d/%02d %02d:%02d:%02d+%02d !\r\n", cnt,
					DT.day, DT.mon, DT.year, DT.hour, DT.min, DT.sec, DT.tz);
		}
	}
*/

	return ret;
}
//-----------------------------------------------------------------------------------------
int8_t parseEvent(char *in, void *g)
{
char *uks = NULL, *uki = NULL;
int8_t id = -1, ret = -1;
int i, j, k;

	gsmFlags_t *gf = (gsmFlags_t *)g;

	for (int8_t i = 0; i < gsmEventMax; i++) {
		if (strstr(in, gsmEvent[i])) {
			id = i;
			break;
		}
	}

	if (id != -1) {
		uks = in + strlen(gsmEvent[id]);
		switch (id) {
			case _RDY:
				gf->rdy = 1;
			break;
			case _CFUN:
				if (*uks == '1') gf->cFun = 1;
				else
				if (*uks == '0') gf->cFun = 0;
			break;
			case _CPIN:
				if (!strcmp(uks, "READY")) gf->cPin = 1; else gf->cPin = 0;
			break;
			case _CallReady:
				gf->cReady = 1;
			break;
			case _SMSReady:
				if (gf->sReady) gf->begin = 1;
				else
				gf->sReady = 1;
			break;
			case _Revision:
				if (strlen(uks) > 2) {
					strncpy(gsmREV, uks, sizeof(gsmREV) - 1);
				}
			break;
			case _CSQ://+CSQ: 14,0
				if ((uki = strchr(uks, ',')) != NULL) *uki = '\0';
				gsmRSSI = dBmRSSI[atoi(uks) & 0x1f];
			break;
			case _CREG://+CREG: 0,1
				if ((uki = strchr(uks, ',')) != NULL) {
					if (*(uki + 1) == '1') gf->reg = 1; else gf->reg = 0;
				}
			break;
			case _CGATT://+CGATT: 1
				if (*uks == '1') gf->cGat = 1;
				else
				if (*uks == '0') gf->cGat = 0;
			break;
			case _CLBS:
			break;
			case _CNTP://+CNTP: 1         || 202.120.2.101,32
				if (strlen(uks) > 4) {
					strncpy(cntpSRV, uks, sizeof(cntpSRV) - 1);
				} else {
					if (*uks == '1') {
						gf->tReady = 1;
						//ret = cCCLK;
					} else {
						gf->tReady = 0;
					}
				}
			break;
			case _CCLK://+CCLK: "21/11/01,12:49:31+02"
				if (strlen(uks) > 12) {
					gf->okDT = 0;
					gf->reqDT = 1;
					uks++;
					char *uki = strchr(uks, '"');
					if (uki) *uki = '\0';
					strncpy(sntpDT, uks, sizeof(sntpDT) - 1);
					/**/
					if (gf->tReady) {
						if (checkDT(sntpDT)) {
							gf->reqDT = 1;
							if (set_DT()) {
								gf->okDT = 1;
								Report(NULL, false,
										"Set date/time %02d/%02d/%02d %02d:%02d:%02d+%02d OK !\r\n",
										DT.day, DT.mon, DT.year, DT.hour, DT.min, DT.sec, DT.tz);
							}
						}
					}
					/**/
				}
			break;
			case _CBC://+CBC: 0,65,3928
				if ((uki = strchr(uks, ',')) != NULL) {
					uks = uki + 1;
					uki = strchr(uks, ',');
					if (uki) VCC = atoi(uki + 1);
				}
			break;
			case _CMEE://+CMEE: 1
				gf->cmee = atoi(uks);
			break;
			case _CUSD://"+CUSD: ",//+CUSD: 0, "003200300030002E003000300020 .... 340023", 72
#ifdef SET_SMS
				uks += 4;//uk to begin ucs2 string
				char *uke = strstr(uks, "\", 72");
				if (uke) {
					if (!cusd) cusd = (char *)calloc(1, SMS_BUF_LEN);
					if (cusd) {
						memset(SMS_text, 0, SMS_BUF_LEN);
						memset(cusd, 0, SMS_BUF_LEN);
						memcpy(cusd, uks, uke - uks);
						if (ucs2_to_utf8(cusd, NULL, (uint8_t *)SMS_text)) Report(NULL, false, "%s\r\n", SMS_text);
					} else devError |= devMem;
				}
#endif
			break;
			case _CMT:
			case _SCLASS0:
#ifdef SET_SMS
				/**/
				gf->sms = 1;
				memset(SMS_text, 0, SMS_BUF_LEN);
				k = strlen(in);
				if (k > SMS_BUF_LEN - 3) k = SMS_BUF_LEN - 3;
				strncpy(SMS_text, in, k);
				strcat(SMS_text, eol);
				//Report(NULL, false, "SMS_text:%s\r\n", in);
				/**/
#endif
			break;
			case _STATE:
			{
				int8_t j = -1;
				while (++j < gsmStateMax) {
					if (strstr(uks, gsmState[j])) {
						gf->state = j;
						break;
					}
				}
				if (j == gsmStateMax)  gf->state = gsmStateMax - 1;
			}
			break;
			case _CONNECTOK:
				gf->connect = gf->busy = 1;
				gf->send = 0;
				gf->fail = gf->closed = 0;
				gf->shut = gf->error = 0;
				HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_RESET);
			break;
			case _CONNECTFAIL:
				gf->fail = 1;
				gf->connect = gf->shut = 0;
				gf->prompt = 0;
				ret = cCIPSHUT;
			break;
			case _CLOSED:
				gf->closed = 1;
				gf->connect = gf->prompt = 0;
				gf->error = 0;
				ret = cCIPSHUT;
			break;
			case _SHUTOK:
				gf->shut = 1;
				gf->connect = gf->prompt = 0;
				gf->error = gf->busy = 0;
				HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_SET);
			break;
			case _PROMPT:
				gf->prompt = 1;
				gf->sendOK = 0;
			break;
			case _SENDOK:
				gf->sendOK = 1;
				HAL_GPIO_WritePin(CON_LED_GPIO_Port, CON_LED_Pin, GPIO_PIN_RESET);
			break;
			case _ERROR:
				gf->error = 1;
				gf->ok = 0;
				if (gf->rlist) gf->rlist = 0;
			break;
			case _OK:
				gf->ok = 1;
				gf->error = 0;
				if (gf->rlist) gf->rlist = 0;
			break;
		}
	} else {
#ifdef SET_SMS
		if (gf->sms) {
			gf->sms = 0;
			//
			j = strlen(SMS_text);
			i = strlen(in);
			if ((j + i + 2) < SMS_BUF_LEN) {
				strcat(SMS_text, in);//strcat(&SMS_text[j], in);
				//if (!strstr(in, eol))
					strcat(SMS_text, eol);
				//if (i) Report(NULL, false, in);
				memset(abcd, 0, sizeof(abcd));
				memset(fromNum, 0, sizeof(fromNum));
				sms_num = 0;
				sms_len = conv_ucs2_text((uint8_t *)SMS_text, fromNum, abcd, 0);
				if (sms_len > 0) {
					Report(NULL, true, "[SMS] len=%u udhi=[%02X%02X%02X%02X%02X] from='%s' body:\r\n%.*s\r\n",
							sms_len, abcd[0], abcd[1], abcd[2], abcd[3], abcd[4], fromNum, sms_len, SMS_text);
					//
					if ((abcd[0] == 1) && abcd[3]) {//with_UDHI and total > 0
						memset((uint8_t *)&reco, 0, sizeof(s_udhi_t));
						memcpy((uint8_t *)&reco, abcd, sizeof(abcd));
						if (reco.total <= maxSMSPart) {
							if (sms_len >= MaxBodyLen) sms_len = MaxBodyLen - 1;
							memcpy(reco.txt, SMS_text, sms_len);
							reco.len = sms_len;
							if (PutSMSList(&reco) != 255) {
								if (!wait_sms) wait_sms = get_tmr(wait_sms_time);//set timer for wait all patrs recv.
								if (LookAllPart(reco.total) == reco.total) {//all parts are present -> concat begin
									*SMS_text = '\0';
									if (ConcatSMS(SMS_text, reco.total, &sms_num, &sms_len) == reco.total) {
										Report(NULL, true, "[SMS] Concat message #%u (len=%u parts=%u) done:\r\n%.*s\r\n",
												sms_num, sms_len, reco.total, sms_len, SMS_text);
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
									InitSMSList();
									wait_sms = 0;
								}
							}
						}
					} else {//without_UDHI
#ifdef SET_SMS_QUEUE
						//----------------  check : command present in sms ?   ------------------------
						checkSMS(SMS_text, fromNum);
						//-----------------------------------------------------------------------------
						//
						*smsTMP = '\0';
						if (!makeSMSString(SMS_text, &sms_len, fromNum, sms_num, smsTMP, sizeof(smsTMP) - 1)) {
							if (addRECQ(smsTMP, &smsq) < 0) {
								Report(NULL, true, smsTMP);
							}
						}
#endif
					}
				}
				//*in = '\0';
			} else Report(NULL, true, "Too long string - %d bytes\r\n", j + i + 2);
		} else if (gf->rlist) {
			int dl = strlen(in);
			if ((dl >= 3) && (dl <= 4)) {
				if (indList < MAX_FREQ_LIST) {
					freqList[indList] = (uint16_t)atoi(in);
				}
				indList++;
			}
		}
#endif
	}

	return ret;
}
//-----------------------------------------------------------------------------------------
void toUppers(char *st)
{
int i;

    for (i = 0; i < strlen(st); i++) *(st + i) = toupper(*(st + i));
}
//-----------------------------------------------------------------------------------------
uint16_t mkData(char *data)
{
s_float_t flo = {0, 0};

	sprintf(data, "{\"dev\":\"%s\"", dev_name);
#ifdef SET_TEMP_SENSOR
	floatPart(temp, &flo); sprintf(data+strlen(data), ",\"temp\":%lu.%lu", flo.cel, flo.dro / 10000);
#endif
#ifdef SET_GPS
	floatPart(GPS.dec_latitude, &flo); sprintf(data+strlen(data),",\"lat\":%02lu.%04lu",  flo.cel, flo.dro / 100);
	floatPart(GPS.dec_longitude,&flo); sprintf(data+strlen(data),",\"lon\":%02lu.%04lu", flo.cel, flo.dro / 100);
	floatPart(GPS.msl_altitude, &flo); sprintf(data+strlen(data),",\"sat\":%d,\"alt\":%lu", GPS.satelites, flo.cel);
	floatPart(GPS.speed_k, &flo);      sprintf(data+strlen(data), ",\"spd\":%lu.%02lu", flo.cel, flo.dro/10000);
	floatPart(GPS.course_d, &flo);     sprintf(data+strlen(data), ",\"dir\":%lu.%02lu", flo.cel, flo.dro/10000);
#endif
	sprintf(data+strlen(data), "}\r\n%c", CTRL_Z);

	return (uint16_t)strlen(data);
}
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------


//******************************************************************************************


