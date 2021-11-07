
#include "hdr.h"


#include "main.h"
#include "func.h"


//******************************************************************************************

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
void set_Date(time_t epoch)
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
struct tm ts;

	gmtime_r(&epoch, &ts);

	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	if (HAL_RTC_SetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
	else {
		sTime.Hours   = ts.tm_hour + tZone;
		sTime.Minutes = ts.tm_min;
		sTime.Seconds = ts.tm_sec;
		if (HAL_RTC_SetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			setDate = true;
		}
	}
}
//------------------------------------------------------------------------------------------
//       Функция возвращает текущее время (в формате epochtime) из модуля RTC
//
uint32_t get_Date()
{
	if (!setDate) return get_tmr(0);

	struct tm ts;

	RTC_TimeTypeDef sTime = {0};
	if (HAL_RTC_GetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
	ts.tm_hour = sTime.Hours;
	ts.tm_min  = sTime.Minutes;
	ts.tm_sec  = sTime.Seconds;

	RTC_DateTypeDef sDate = {0};
	if (HAL_RTC_GetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) return get_tmr(0);
	ts.tm_wday = sDate.WeekDay;
	ts.tm_mon  = sDate.Month - 1;
	ts.tm_mday = sDate.Date;
	ts.tm_year = sDate.Year;

	return ((uint32_t)mktime(&ts));
}
//------------------------------------------------------------------------------------
//   Функция формирует символьную строку с датой и временем из значения epochtime
//         и возвращает длинну сформированной символьной строки
//
int sec_to_str_time(uint32_t sec, char *stx)
{
int ret = 0;

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
		if (HAL_RTC_GetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			if (HAL_RTC_GetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;////errLedOn(__func__);
			else {
				ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u",
								   sDate.Date, sDate.Month,
								   sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
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

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	if (HAL_RTC_GetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
	else {
		if (HAL_RTC_GetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;//errLedOn(__func__);
		else {
			ret = sprintf(stx, "%02u.%02u %02u:%02u:%02u ",
							sDate.Date, sDate.Month,
							sTime.Hours, sTime.Minutes, sTime.Seconds);
		}
	}

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
		if (HAL_UART_Transmit_DMA(portLOG, (uint8_t *)buff, strlen(buff)) != HAL_OK) devError |= devUART;
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
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

	sDate.WeekDay = 0;
	sDate.Month   = DT.mon;
	sDate.Date    = DT.day;
	sDate.Year    = DT.year;
	if (HAL_RTC_SetDate(portRTC, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
	else {
		sTime.Hours   = DT.hour;// + DT.tz;
		sTime.Minutes = DT.min;
		sTime.Seconds = DT.sec;
		if (HAL_RTC_SetTime(portRTC, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
		else {
			setDate = true;
			ret = true;
		}
	}

	return ret;
}
//------------------------------------------------------------------------------------------
void prnFlags(void *g)
{
	gsmFlags_t *gf = (gsmFlags_t *)g;

	Report(NULL,
		   true,
		   "Flags:\n\trdy:%u\n\tcFun:%u\n\tcPin:%u\n\tCallReady:%u\n\tSMSReady:%u\n\treg:%u\n\tcGat:%u\n\tcmee:%u\n"
		   "\tcntp:%u\n\tokDT:%u\n\tstate:%u\n\tconnect:%u\n\terror:%u\n\tok:%u\n"
		   "\tsntpSRV:'%s'\n\tsntpDT:'%s'\n\timei:%s\n\tVcc:%u\r\n",
		   gf->rdy, gf->cFun, gf->cPin, gf->cReady, gf->sReady, gf->reg, gf->cGat, gf->cmee,
		   gf->tReady, gf->okDT, gf->state, gf->connect, gf->error, gf->ok,
		   cntpSRV, sntpDT, gsmIMEI, VCC);
}
//------------------------------------------------------------------------------------------
bool checkDT(char *str)//21/11/01,12:49:31+02
{
bool ret = false;

	if (strchr(str, '+')) {
		if (sscanf(str, "%d/%d/%d,%d:%d:%d+%d", &DT.year, &DT.mon, &DT.day, &DT.hour, &DT.min, &DT.sec, &DT.tz) >= 1) ret = true;
	} else if (strchr(str, '-')) {
		if (sscanf(str, "%d/%d/%d,%d:%d:%d-%d", &DT.year, &DT.mon, &DT.day, &DT.hour, &DT.min, &DT.sec, &DT.tz) >= 1) ret = true;
	}

	return ret;
}
//-----------------------------------------------------------------------------------------
int8_t parseEvent(char *in, void *g)
{
char *uks = NULL, *uki = NULL;
int8_t id = -1, ret = -1;

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
					uks++;
					char *uki = strchr(uks, '"');
					if (uki) *uki = '\0';
					strncpy(sntpDT, uks, sizeof(sntpDT) - 1);
					if (gf->tReady) {
						if (checkDT(sntpDT)) {
							gf->reqDT = 1;
							if (set_DT()) gf->okDT = 1;
						}
					}
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
				if (cusd) {
					free(cusd);
					cusd = NULL;
				}
				cusd = (char *)calloc(1, MAX_SMS_BUF);
				if (cusd) {
					uks += 4;//uk to begin ucs2 string
					char *uke = strstr(uks, "\", 72");
					if (uke) {
						memset(cusd, 0, MAX_SMS_BUF);
						memcpy(cusd, uks, uke - uks);
						if (ucs2_to_utf8(cusd, NULL, (uint8_t *)SMS_text)) Report(NULL, false, "%s\r\n", SMS_text);
					}
					free(cusd); cusd = NULL;
				} else devError |= devMem;
			break;
			case _STATE:
			{
				int8_t j =-1;
				while(++j < gsmStateMax) {
					if (strstr(uks, gsmState[j])) {
						gf->state = j;
						break;
					}
				}
			}
			break;
			case _CONNECTOK:
				gf->connect = 1;
			break;
			case _ERROR:
				gf->error = 1;
				gf->ok = 0;
			break;
			case _OK:
				gf->ok = 1;
				gf->error = 0;
			break;
		}
	}

	return ret;
}
//------------------------------------------------------------------------------------------
uint8_t hextobin(char st, char ml)
{
const char hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t a = 255, b, c, i;

	for	(i = 0; i < 16; i++) { if (st == hex[i]) { b = i; break; } else b = 255; }
	for	(i = 0; i < 16; i++) { if (ml == hex[i]) { c = i; break; } else c = 255; }
	if ((b != 255) && (c != 255)) { b = b << 4;   a = b | c; }

	return a;
}
//-----------------------------------------------------------------------------------------
int ucs2_to_utf8(char *buf_in, uint8_t *udl, uint8_t *utf8)
{
int ret = 0, i = 0, len;
uint8_t a, b;
char *ptr = buf_in;
uint8_t *out = utf8;
uint16_t ucs2;


	if (!udl) len = strlen(buf_in) >> 2; else len = *udl >> 1;

    while (i < len) {
    	a = hextobin(*ptr, *(ptr + 1));   ptr += 2;
    	b = hextobin(*ptr, *(ptr + 1));   ptr += 2;
    	ucs2 = a;   ucs2 <<= 8;   ucs2 |= b;
    	if (ucs2 < 0x80) {
    		*out++ = (uint8_t)ucs2;
    		ret++;
    	} else {
    		*out++ = (uint8_t)((ucs2 >> 6)   | 0xC0);
    		*out++ = (uint8_t)((ucs2 & 0x3F) | 0x80);
    		ret += 2;
    	}
    	i++;
    }

    return ret;
}
//-----------------------------------------------------------------------------------------


//******************************************************************************************


