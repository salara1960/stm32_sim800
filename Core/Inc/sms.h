#ifndef __SMS_H__
#define __SMS_H__

#include "hdr.h"
#include "main.h"

#ifdef SET_SMS


#define SMS_BUF_LEN 640//512
#define maxSMSPart 8
#define MaxBodyLen 161

#define cod_PDU_len 159
#define lenFrom len_From//32
#define wait_sms_time 60 * 3
#define max_smsType 3

#pragma pack(push,1)
typedef struct s_udhi_t {
	uint8_t tp;    // 0-обычная смс, 1-часть длинной смс, 255-квитанция
	uint16_t num;  // индекс (номер) смс
	uint8_t total; // количество частей
	uint8_t part;  // номер части
	uint16_t len;
	char txt[MaxBodyLen];
} s_udhi_t;
#pragma pack(pop)

//------------------------------------------------------------------------------------------

uint32_t infCounter;
char SMS_text[SMS_BUF_LEN];
#ifdef SET_SMS_QUEUE
	char smsTMP[MAX_UART_BUF];
#endif

//------------------------------------------------------------------------------------------

uint8_t hextobin(char st, char ml);
int ucs2_to_utf8(char *buf_in, uint8_t *udl, uint8_t *utf8);
int conv_ucs2_text(uint8_t *buffer_txt, char *fromik, uint8_t *udhi5, uint8_t prn);
void InitSMSList();
uint8_t PutSMSList(s_udhi_t *rec);
uint8_t LookAllPart(uint8_t total);
uint8_t ConcatSMS(char *buf, uint8_t total, uint16_t *sn, uint16_t *sl);
uint8_t getSMSTotalCounter();
int gsm7bit_to_text(int len_inbuff, uint8_t *inbuff, uint8_t *outbuff, int fl, uint8_t max_udl, uint8_t u_len);
#ifdef SET_SMS_QUEUE
	int8_t makeSMSString(const char *body, uint16_t *blen, char *fnum, uint16_t snum, char *buf, int max_len_buf);
	void checkSMS(char *body, char *from);
#endif
//------------------------------------------------------------------------------------------


#endif

#endif /* __SMS_H__ */
