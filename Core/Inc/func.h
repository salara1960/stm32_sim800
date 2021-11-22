#ifndef __FUNC_H__
#define __FUNC_H__

#include "hdr.h"
#include "main.h"

//------------------------------------------------------------------------
#define REC_BUF_LEN MAX_UART_BUF


#define MAX_QREC 16//64//16
#define MAX_SQREC 8//64//16


//#ifdef SET_STATIC_MEM
//	char PrnBuf[MAX_UART_BUF];// Служебный буфер для функции Report()
//#endif

//------------------------------------------------------------------------

#ifdef SET_FLOAT_PART
typedef struct {
	uint32_t cel;
	uint32_t dro;
} s_float_t;
#endif

#pragma pack(push,1)
typedef struct q_rec_t {
	int8_t id;
	char *adr;
} q_rec_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct s_recq_t {
	volatile uint8_t lock;
	uint8_t put;
	uint8_t get;
	q_rec_t rec[MAX_QREC];
} s_recq_t;
#pragma pack(pop)


//------------------------------------------------------------------------

#ifdef USED_FREERTOS
	uint32_t waitRTC;
#endif

extern volatile bool setDate;
extern volatile uint32_t extDate;
extern uint8_t uartRdy;
extern volatile uint32_t secCounter;
extern volatile uint32_t HalfSecCounter;
extern int8_t tZone;

//------------------------------------------------------------------------
#ifdef SET_FLOAT_PART
	extern void floatPart(float val, s_float_t *part);
#endif

uint32_t get_secCounter();
void inc_secCounter();
uint32_t get_hsCounter();
void inc_hsCounter();
uint32_t get_tmr10(uint32_t ms);
bool check_tmr10(uint32_t ms);
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
uint64_t get_hstmr(uint64_t hs);
bool check_hstmr(uint64_t hs);

void gsmReset();

#ifdef SET_SMS
	#ifdef SET_SMS_QUEUE
		bool initSRECQ(s_recq_t *q);
		void clearSRECQ(s_recq_t *q);
		int8_t putSRECQ(char *adr, s_recq_t *q);
		int8_t getSRECQ(char *dat, s_recq_t *q);
		int8_t addSRECQ(char *txt, s_recq_t *q);
	#endif
#endif

bool initRECQ(s_recq_t *q);//s_recq_t recq;
bool clearRECQ(s_recq_t *q);
int8_t putRECQ(char *adr, s_recq_t *q);
int8_t getRECQ(char *dat, s_recq_t *q);
void errLedOn(const char *from);
void set_Date(time_t epoch);
uint32_t get_Date();
int sec_to_str_time(uint32_t sec, char *stx);
int sec_to_string(uint32_t sec, char *stx);
uint8_t Report(const char *tag, bool addTime, const char *fmt, ...);
bool set_DT();
void prnFlags(void *g);
void prnRList();
bool checkDT(char *str);
int8_t parseEvent(char *in, void *g);
#ifdef SET_SMS
	//extern uint8_t hextobin(char st, char ml);
	extern int ucs2_to_utf8(char *buf_in, uint8_t *udl, uint8_t *utf8);
#endif
void toUppers(char *st);
uint16_t mkData(char *data);

//------------------------------------------------------------------------

#endif /* __FUNC_H__ */

