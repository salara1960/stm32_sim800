/*
 * ds18b20.h
 *
 *  Created on: Nov 10, 2021
 *      Author: alarm
 */

#ifndef __DS18B20_H_
#define __DS18B20_H_

#include "hdr.h"
#include "main.h"

#ifdef SET_TEMP_SENSOR

//------------------------------------------------------------------------
#define _DS18B20_MAX_SENSORS 1
#define _DS18B20_USE_FREERTOS 1

#define	_DS18B20_CONVERT_TIMEOUT_MS	5000
#define	_DS18B20_UPDATE_INTERVAL_MS	10000	//  (((	if==0  >> Ds18b20_ManualConvert()  )))    ((( if>0  >>>> Auto refresh )))

#define DS18B20_FAMILY_CODE			0x28
#define DS18B20_CMD_ALARMSEARCH		0xEC

//   DS18B20 read temperature command
#define DS18B20_CMD_CONVERTTEMP		0x44 	// Convert temperature
#define DS18B20_DECIMAL_STEPS_12BIT	0.0625
#define DS18B20_DECIMAL_STEPS_11BIT	0.125
#define DS18B20_DECIMAL_STEPS_10BIT	0.25
#define DS18B20_DECIMAL_STEPS_9BIT	0.5

//   Bits locations for resolution
#define DS18B20_RESOLUTION_R1		6
#define DS18B20_RESOLUTION_R0		5

//           CRC enabled
#ifdef DS18B20_USE_CRC
	#define DS18B20_DATA_LEN		9
#else
	#define DS18B20_DATA_LEN		2
#endif


#if (_DS18B20_USE_FREERTOS==1)
//	#include "cmsis_os.h"
	#define	OneWireDelay(x)	osDelay(x)
#else
	#define	OneWireDelay(x)	HAL_Delay(x)
#endif
#define	Ds18b20Delay(x) OneWireDelay(x)



//          OneWire commands
#define ONEWIRE_CMD_RSCRATCHPAD		0xBE
#define ONEWIRE_CMD_WSCRATCHPAD		0x4E
#define ONEWIRE_CMD_CPYSCRATCHPAD	0x48
#define ONEWIRE_CMD_RECEEPROM		0xB8
#define ONEWIRE_CMD_RPWRSUPPLY		0xB4
#define ONEWIRE_CMD_SEARCHROM		0xF0
#define ONEWIRE_CMD_READROM			0x33
#define ONEWIRE_CMD_MATCHROM		0x55
#define ONEWIRE_CMD_SKIPROM			0xCC

//------------------------------------------------------------------------

typedef enum {
	DS18B20_Resolution_9bits = 9,
	DS18B20_Resolution_10bits,
	DS18B20_Resolution_11bits,
	DS18B20_Resolution_12bits
} DS18B20_Resolution_t;

#pragma pack(push,1)
typedef struct {
	uint8_t Address[8];
	float 	Temperature;
	bool	DataIsValid;
} Ds18b20Sensor_t;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct {
	GPIO_TypeDef* GPIOx;           // GPIOx port to be used for I/O functions
	uint16_t GPIO_Pin;             // GPIO Pin to be used for I/O functions
	uint8_t LastDiscrepancy;       // Search private
	uint8_t LastFamilyDiscrepancy; // Search private
	uint8_t LastDeviceFlag;        // Search private
	uint8_t ROM_NO[8];             // 8-bytes address of last search device
} OneWire_t;
#pragma pack(pop)

//------------------------------------------------------------------------

extern Ds18b20Sensor_t	ds18b20[_DS18B20_MAX_SENSORS];
OneWire_t	OneWire;
uint8_t	  	OneWireDevices;
uint8_t 	TempSensorCount;
uint8_t		Ds18b20StartConvert;
uint16_t	Ds18b20Timeout;

float fTemp;
bool sensPresent;


bool Ds18b20_ManualConvert(void);


//        OneWire delay
void ONEWIRE_DELAY(uint16_t time_us);
//        Pin settings
void ONEWIRE_LOW(OneWire_t *gp);
void ONEWIRE_HIGH(OneWire_t *gp);
void ONEWIRE_INPUT(OneWire_t *gp);
void ONEWIRE_OUTPUT(OneWire_t *gp);

//------------------------------------------------------------------------

uint8_t DS18B20_Start(OneWire_t *OneWireStruct, uint8_t *ROM);
void 	DS18B20_StartAll(OneWire_t *OneWireStruct);
bool	DS18B20_Read(OneWire_t *OneWireStruct, uint8_t *ROM, float *destination);
uint8_t DS18B20_GetResolution(OneWire_t *OneWireStruct, uint8_t *ROM);
uint8_t DS18B20_SetResolution(OneWire_t *OneWireStruct, uint8_t *ROM, DS18B20_Resolution_t resolution);
uint8_t DS18B20_Is(uint8_t *ROM);
uint8_t DS18B20_SetAlarmHighTemperature(OneWire_t *OneWireStruct, uint8_t *ROM, int8_t temp);
uint8_t DS18B20_SetAlarmLowTemperature(OneWire_t *OneWireStruct, uint8_t *ROM, int8_t temp);
uint8_t DS18B20_DisableAlarmTemperature(OneWire_t *OneWireStruct, uint8_t *ROM);
uint8_t DS18B20_AlarmSearch(OneWire_t *OneWireStruct);
uint8_t DS18B20_AllDone(OneWire_t *OneWireStruct);


void OneWire_Init(OneWire_t *OneWireStruct, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t OneWire_Reset(OneWire_t *OneWireStruct);
uint8_t OneWire_ReadByte(OneWire_t *OneWireStruct);
void OneWire_WriteByte(OneWire_t *OneWireStruct, uint8_t byte);
void OneWire_WriteBit(OneWire_t *OneWireStruct, uint8_t bit);
uint8_t OneWire_ReadBit(OneWire_t *OneWireStruct);
uint8_t OneWire_Search(OneWire_t *OneWireStruct, uint8_t command);
void OneWire_ResetSearch(OneWire_t *OneWireStruct);
uint8_t OneWire_First(OneWire_t *OneWireStruct);
uint8_t OneWire_Next(OneWire_t *OneWireStruct);
void OneWire_GetFullROM(OneWire_t *OneWireStruct, uint8_t *firstIndex);
void OneWire_Select(OneWire_t *OneWireStruct, uint8_t *addr);
void OneWire_SelectWithPointer(OneWire_t *OneWireStruct, uint8_t *ROM);
uint8_t OneWire_CRC8(uint8_t *addr, uint8_t len);

//------------------------------------------------------------------------

#endif

#endif /* INC_DS18B20_H_ */
