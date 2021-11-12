/*
 * ds18b20.c
 *
 *  Created on: Nov 10, 2021
 *      Author: alarm
 */


#include "hdr.h"

#include "main.h"
#include "ds18b20.h"

#ifdef SET_TEMP_SENSOR

//******************************************************************************************
Ds18b20Sensor_t	ds18b20[_DS18B20_MAX_SENSORS];

OneWire_t	OneWire;
uint8_t	  	OneWireDevices;
uint8_t 	TempSensorCount = 0;
uint8_t		Ds18b20StartConvert = 0;
uint16_t	Ds18b20Timeout = 0;

//	osThreadId 	Ds18b20Handle;
//	void Task_Ds18b20(void const * argument);

float fTemp = 0.0;
bool sensPresent = false;

//******************************************************************************************

void ONEWIRE_DELAY(uint16_t time_us)
{
	tmrDS18B20->Instance->CNT = 0;
	while (tmrDS18B20->Instance->CNT <= time_us);
}
//----------------------------------------------------------------------------------
void ONEWIRE_LOW(OneWire_t *gp)
{
	gp->GPIOx->BSRR = gp->GPIO_Pin<<16;
}
//----------------------------------------------------------------------------------
void ONEWIRE_HIGH(OneWire_t *gp)
{
	gp->GPIOx->BSRR = gp->GPIO_Pin;
}
//----------------------------------------------------------------------------------
void ONEWIRE_INPUT(OneWire_t *gp)
{
	GPIO_InitTypeDef	gpinit;
	gpinit.Mode = GPIO_MODE_INPUT;
	gpinit.Pull = GPIO_NOPULL;
	gpinit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpinit.Pin = gp->GPIO_Pin;
	HAL_GPIO_Init(gp->GPIOx, &gpinit);
}
//----------------------------------------------------------------------------------
void ONEWIRE_OUTPUT(OneWire_t *gp)
{
	GPIO_InitTypeDef	gpinit;
	gpinit.Mode = GPIO_MODE_OUTPUT_OD;
	gpinit.Pull = GPIO_NOPULL;
	gpinit.Speed = GPIO_SPEED_FREQ_HIGH;
	gpinit.Pin = gp->GPIO_Pin;
	HAL_GPIO_Init(gp->GPIOx, &gpinit);
}
//----------------------------------------------------------------------------------
void OneWire_Init(OneWire_t *OneWireStruct, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	HAL_TIM_Base_Start(tmrDS18B20);

	OneWireStruct->GPIOx = GPIOx;
	OneWireStruct->GPIO_Pin = GPIO_Pin;
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_HIGH(OneWireStruct);
	OneWireDelay(1000);
	ONEWIRE_LOW(OneWireStruct);
	OneWireDelay(1000);
	ONEWIRE_HIGH(OneWireStruct);
	OneWireDelay(2000);
}
//----------------------------------------------------------------------------------
inline uint8_t OneWire_Reset(OneWire_t* OneWireStruct)
{
	uint8_t i;

	// Line low, and wait 480us
	ONEWIRE_LOW(OneWireStruct);
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_DELAY(480);
	ONEWIRE_DELAY(20);
	// Release line and wait for 70us
	ONEWIRE_INPUT(OneWireStruct);
	ONEWIRE_DELAY(70);
	// Check bit value
	i = HAL_GPIO_ReadPin(OneWireStruct->GPIOx, OneWireStruct->GPIO_Pin);

	// Delay for 410 us
	ONEWIRE_DELAY(410);
	// Return value of presence pulse, 0 = OK, 1 = ERROR

	return i;
}
//----------------------------------------------------------------------------------
inline void OneWire_WriteBit(OneWire_t* OneWireStruct, uint8_t bit)
{
	if (bit) {
		// Set line low
		ONEWIRE_LOW(OneWireStruct);
		ONEWIRE_OUTPUT(OneWireStruct);
		ONEWIRE_DELAY(10);

		// Bit high
		ONEWIRE_INPUT(OneWireStruct);

		// Wait for 55 us and release the line
		ONEWIRE_DELAY(55);
		ONEWIRE_INPUT(OneWireStruct);
	} else {
		// Set line low
		ONEWIRE_LOW(OneWireStruct);
		ONEWIRE_OUTPUT(OneWireStruct);
		ONEWIRE_DELAY(65);

		// Bit high
		ONEWIRE_INPUT(OneWireStruct);

		// Wait for 5 us and release the line
		ONEWIRE_DELAY(5);
		ONEWIRE_INPUT(OneWireStruct);
	}

}
//----------------------------------------------------------------------------------
inline uint8_t OneWire_ReadBit(OneWire_t* OneWireStruct)
{
uint8_t bit = 0;

	// Line low
	ONEWIRE_LOW(OneWireStruct);
	ONEWIRE_OUTPUT(OneWireStruct);
	ONEWIRE_DELAY(2);

	// Release line
	ONEWIRE_INPUT(OneWireStruct);
	ONEWIRE_DELAY(10);

	// Read line value
	if (HAL_GPIO_ReadPin(OneWireStruct->GPIOx, OneWireStruct->GPIO_Pin)) bit = 1;// Bit is HIGH

	// Wait 50us to complete 60us period
	ONEWIRE_DELAY(50);

	// Return bit value

	return bit;
}
//----------------------------------------------------------------------------------
void OneWire_WriteByte(OneWire_t* OneWireStruct, uint8_t byte)
{
uint8_t i = 8;

	// Write 8 bits
	while (i--) {
		// LSB bit is first
		OneWire_WriteBit(OneWireStruct, byte & 0x01);
		byte >>= 1;
	}
}
//----------------------------------------------------------------------------------
uint8_t OneWire_ReadByte(OneWire_t *OneWireStruct)
{
uint8_t i = 8, byte = 0;

	while (i--) {
		byte >>= 1;
		byte |= (OneWire_ReadBit(OneWireStruct) << 7);
	}

	return byte;
}
//----------------------------------------------------------------------------------
uint8_t OneWire_First(OneWire_t *OneWireStruct)
{
	OneWire_ResetSearch(OneWireStruct);// Reset search values

	return OneWire_Search(OneWireStruct, ONEWIRE_CMD_SEARCHROM);// Start with searching
}
//----------------------------------------------------------------------------------
uint8_t OneWire_Next(OneWire_t *OneWireStruct)
{
   // Leave the search state alone
   return OneWire_Search(OneWireStruct, ONEWIRE_CMD_SEARCHROM);
}
//----------------------------------------------------------------------------------
void OneWire_ResetSearch(OneWire_t *OneWireStruct)
{
	// Reset the search state
	OneWireStruct->LastDiscrepancy = 0;
	OneWireStruct->LastDeviceFlag = 0;
	OneWireStruct->LastFamilyDiscrepancy = 0;
}
//----------------------------------------------------------------------------------
uint8_t OneWire_Search(OneWire_t *OneWireStruct, uint8_t command)
{
uint8_t id_bit_number = 1;
uint8_t last_zero = 0, rom_byte_number = 0, search_result = 0;
uint8_t id_bit, cmp_id_bit;
uint8_t rom_byte_mask = 1, search_direction;

	// if the last call was not the last one
	if (!OneWireStruct->LastDeviceFlag) {
		// 1-Wire reset
		if (OneWire_Reset(OneWireStruct)) {
			// Reset the search
			OneWireStruct->LastDiscrepancy = 0;
			OneWireStruct->LastDeviceFlag = 0;
			OneWireStruct->LastFamilyDiscrepancy = 0;
			return 0;
		}

		// issue the search command
		OneWire_WriteByte(OneWireStruct, command);

		// loop to do the search
		do {
			// read a bit and its complement
			id_bit = OneWire_ReadBit(OneWireStruct);
			cmp_id_bit = OneWire_ReadBit(OneWireStruct);

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1)) {
				break;
			} else {
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit) {
					search_direction = id_bit;  // bit write value for search
				} else {
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < OneWireStruct->LastDiscrepancy) {
						search_direction = ((OneWireStruct->ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					} else {
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == OneWireStruct->LastDiscrepancy);
					}

					// if 0 was picked then record its position in LastZero
					if (!search_direction) {
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9) OneWireStruct->LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1) OneWireStruct->ROM_NO[rom_byte_number] |= rom_byte_mask;
									  else OneWireStruct->ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				OneWire_WriteBit(OneWireStruct, search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (!rom_byte_mask) {
					//docrc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65)) {
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			OneWireStruct->LastDiscrepancy = last_zero;

			// check for last device
			if (!OneWireStruct->LastDiscrepancy) OneWireStruct->LastDeviceFlag = 1;

			search_result = 1;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !OneWireStruct->ROM_NO[0]) {
		OneWireStruct->LastDiscrepancy = 0;
		OneWireStruct->LastDeviceFlag = 0;
		OneWireStruct->LastFamilyDiscrepancy = 0;
		search_result = 0;
	}

	return search_result;
}
//----------------------------------------------------------------------------------
int OneWire_Verify(OneWire_t *OneWireStruct)
{
unsigned char rom_backup[8];
int i,rslt,ld_backup,ldf_backup,lfd_backup;

	// keep a backup copy of the current state
	for (i = 0; i < 8; i++) rom_backup[i] = OneWireStruct->ROM_NO[i];
	ld_backup = OneWireStruct->LastDiscrepancy;
	ldf_backup = OneWireStruct->LastDeviceFlag;
	lfd_backup = OneWireStruct->LastFamilyDiscrepancy;

	// set search to find the same device
	OneWireStruct->LastDiscrepancy = 64;
	OneWireStruct->LastDeviceFlag = 0;

	if (OneWire_Search(OneWireStruct, ONEWIRE_CMD_SEARCHROM)) {
		// check if same device found
		rslt = 1;
		for (i = 0; i < 8; i++) {
			if (rom_backup[i] != OneWireStruct->ROM_NO[i]) {
				rslt = 1;
				break;
			}
		}
	} else rslt = 0;

	// restore the search state
	for (i = 0; i < 8; i++) OneWireStruct->ROM_NO[i] = rom_backup[i];
	OneWireStruct->LastDiscrepancy = ld_backup;
	OneWireStruct->LastDeviceFlag = ldf_backup;
	OneWireStruct->LastFamilyDiscrepancy = lfd_backup;

	// return the result of the verify
	return rslt;
}
//----------------------------------------------------------------------------------
void OneWire_TargetSetup(OneWire_t *OneWireStruct, uint8_t family_code)
{
uint8_t i;

	// set the search state to find SearchFamily type devices
	OneWireStruct->ROM_NO[0] = family_code;
	for (i = 1; i < 8; i++) OneWireStruct->ROM_NO[i] = 0;

	OneWireStruct->LastDiscrepancy = 64;
	OneWireStruct->LastFamilyDiscrepancy = 0;
	OneWireStruct->LastDeviceFlag = 0;
}
//----------------------------------------------------------------------------------
void OneWire_FamilySkipSetup(OneWire_t *OneWireStruct)
{
	// set the Last discrepancy to last family discrepancy
	OneWireStruct->LastDiscrepancy = OneWireStruct->LastFamilyDiscrepancy;
	OneWireStruct->LastFamilyDiscrepancy = 0;

	// check for end of list
	if (OneWireStruct->LastDiscrepancy == 0) OneWireStruct->LastDeviceFlag = 1;
}
//----------------------------------------------------------------------------------
uint8_t OneWire_GetROM(OneWire_t *OneWireStruct, uint8_t index)
{
	return OneWireStruct->ROM_NO[index];
}
//----------------------------------------------------------------------------------
void OneWire_Select(OneWire_t *OneWireStruct, uint8_t *addr)
{
uint8_t i;

	OneWire_WriteByte(OneWireStruct, ONEWIRE_CMD_MATCHROM);

	for (i = 0; i < 8; i++) OneWire_WriteByte(OneWireStruct, *(addr + i));
}
//----------------------------------------------------------------------------------
void OneWire_SelectWithPointer(OneWire_t *OneWireStruct, uint8_t *ROM)
{
uint8_t i;

	OneWire_WriteByte(OneWireStruct, ONEWIRE_CMD_MATCHROM);

	for (i = 0; i < 8; i++) OneWire_WriteByte(OneWireStruct, *(ROM + i));
}
//----------------------------------------------------------------------------------
void OneWire_GetFullROM(OneWire_t *OneWireStruct, uint8_t *firstIndex)
{
uint8_t i;

	for (i = 0; i < 8; i++) *(firstIndex + i) = OneWireStruct->ROM_NO[i];
}
//----------------------------------------------------------------------------------
uint8_t OneWire_CRC8(uint8_t *addr, uint8_t len)
{
uint8_t crc = 0, inbyte, i, mix;

	while (len--) {
		inbyte = *addr++;
		for (i = 8; i; i--) {
			mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}

	// Return calculated CRC
	return crc;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
/*
#if (_DS18B20_USE_FREERTOS==1)
void	Ds18b20_Init(osPriority Priority)
{
	osThreadDef(myTask_Ds18b20, Task_Ds18b20, Priority, 0, 128);
  Ds18b20Handle = osThreadCreate(osThread(myTask_Ds18b20), NULL);
}
#else
bool	Ds18b20_Init(void)
{
	uint8_t	Ds18b20TryToFind=5;
	do
	{
		OneWire_Init(&OneWire,_DS18B20_GPIO ,_DS18B20_PIN);
		TempSensorCount = 0;
		while(HAL_GetTick() < 3000)
			Ds18b20Delay(100);
		OneWireDevices = OneWire_First(&OneWire);
		while (OneWireDevices)
		{
			Ds18b20Delay(100);
			TempSensorCount++;
			OneWire_GetFullROM(&OneWire, ds18b20[TempSensorCount-1].Address);
			OneWireDevices = OneWire_Next(&OneWire);
		}
		if(TempSensorCount>0)
			break;
		Ds18b20TryToFind--;
	}while(Ds18b20TryToFind>0);
	if(Ds18b20TryToFind==0)
		return false;
	for (uint8_t i = 0; i < TempSensorCount; i++)
	{
		Ds18b20Delay(50);
    DS18B20_SetResolution(&OneWire, ds18b20[i].Address, DS18B20_Resolution_12bits);
		Ds18b20Delay(50);
    DS18B20_DisableAlarmTemperature(&OneWire,  ds18b20[i].Address);
  }
	return true;
}
#endif
*/
//----------------------------------------------------------------------------------
bool	Ds18b20_ManualConvert(void)
{
	Ds18b20StartConvert = 1;

	while (Ds18b20StartConvert == 1) Ds18b20Delay(10);
	if (!Ds18b20Timeout) return false;
					else return true;
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_Start(OneWire_t *OneWire, uint8_t *ROM)
{
	// Check if device is DS18B20
	if (!DS18B20_Is(ROM)) return 0;


	OneWire_Reset(OneWire);// Reset line

	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number

	// Start temperature conversion
	OneWire_WriteByte(OneWire, DS18B20_CMD_CONVERTTEMP);

	return 1;
}
//----------------------------------------------------------------------------------
void DS18B20_StartAll(OneWire_t *OneWire)
{
	OneWire_Reset(OneWire);// Reset pulse

	OneWire_WriteByte(OneWire, ONEWIRE_CMD_SKIPROM);// Skip rom

	// Start conversion on all connected devices
	OneWire_WriteByte(OneWire, DS18B20_CMD_CONVERTTEMP);
}
//----------------------------------------------------------------------------------
bool DS18B20_Read(OneWire_t *OneWire, uint8_t *ROM, float *destination)
{
uint16_t temperature;
uint8_t resolution;
int8_t digit, minus = 0;
float decimal;
uint8_t i = 0;
uint8_t data[9];
uint8_t crc;


	if (!DS18B20_Is(ROM)) return false;// Check if device is DS18B20

	// Check if line is released, if it is, then conversion is complete
	if (!OneWire_ReadBit(OneWire)) return false;// Conversion is not finished yet



	OneWire_Reset(OneWire);// Reset line

	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number

	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);// Read scratchpad command by onewire protocol

	// Get data
	for (i = 0; i < 9; i++) data[i] = OneWire_ReadByte(OneWire);// Read byte by byte

	crc = OneWire_CRC8(data, 8);// Calculate CRC

	// Check if CRC is ok
	if (crc != data[8]) return 0;// CRC invalid


	// First two bytes of scratchpad are temperature values
	temperature = data[0] | (data[1] << 8);


	OneWire_Reset(OneWire);// Reset line

	// Check if temperature is negative
	if (temperature & 0x8000) {
		// Two's complement, temperature is negative
		temperature = ~temperature + 1;
		minus = 1;
	}

	// Get sensor resolution
	resolution = ((data[4] & 0x60) >> 5) + 9;

	// Store temperature integer digits and decimal digits
	digit = temperature >> 4;
	digit |= ((temperature >> 8) & 0x7) << 4;

	// Store decimal digits
	switch (resolution) {
		case 9:
			decimal = (temperature >> 3) & 0x01;
			decimal *= (float)DS18B20_DECIMAL_STEPS_9BIT;
		break;
		case 10:
			decimal = (temperature >> 2) & 0x03;
			decimal *= (float)DS18B20_DECIMAL_STEPS_10BIT;
		 break;
		case 11:
			decimal = (temperature >> 1) & 0x07;
			decimal *= (float)DS18B20_DECIMAL_STEPS_11BIT;
		break;
		case 12:
			decimal = temperature & 0x0F;
			decimal *= (float)DS18B20_DECIMAL_STEPS_12BIT;
		 break;
		default:
			decimal = 0xFF;
			digit = 0;
	}

	// Check for negative part
	decimal = digit + decimal;
	if (minus) decimal = 0 - decimal;

	*destination = decimal;// Set to pointer

	return true;// Return 1, temperature valid
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_GetResolution(OneWire_t* OneWire, uint8_t *ROM)
{
uint8_t conf;

	if (!DS18B20_Is(ROM)) return 0;


	OneWire_Reset(OneWire);// Reset line

	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number

	// Read scratchpad command by onewire protocol
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);

	// Ignore first 4 bytes
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);

	// 5th byte of scratchpad is configuration register
	conf = OneWire_ReadByte(OneWire);

	return ((conf & 0x60) >> 5) + 9;// Return 9 - 12 value according to number of bits
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_SetResolution(OneWire_t *OneWire, uint8_t *ROM, DS18B20_Resolution_t resolution)
{
	if (!DS18B20_Is(ROM)) return 0;

	OneWire_Reset(OneWire);// Reset line

	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number

	// Read scratchpad command by onewire protocol
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);

	// Ignore first 2 bytes
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);

	uint8_t th = OneWire_ReadByte(OneWire);
	uint8_t tl = OneWire_ReadByte(OneWire);
	uint8_t conf = OneWire_ReadByte(OneWire);

	if (resolution == DS18B20_Resolution_9bits) {
		conf &= ~(1 << DS18B20_RESOLUTION_R1);
		conf &= ~(1 << DS18B20_RESOLUTION_R0);
	} else if (resolution == DS18B20_Resolution_10bits) {
		conf &= ~(1 << DS18B20_RESOLUTION_R1);
		conf |= 1 << DS18B20_RESOLUTION_R0;
	} else if (resolution == DS18B20_Resolution_11bits) {
		conf |= 1 << DS18B20_RESOLUTION_R1;
		conf &= ~(1 << DS18B20_RESOLUTION_R0);
	} else if (resolution == DS18B20_Resolution_12bits) {
		conf |= 1 << DS18B20_RESOLUTION_R1;
		conf |= 1 << DS18B20_RESOLUTION_R0;
	}


	OneWire_Reset(OneWire);// Reset line

	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number

	// Write scratchpad command by onewire protocol, only th, tl and conf register can be written
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);

	// Write bytes
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);


	OneWire_Reset(OneWire);// Reset line

	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Copy scratchpad to EEPROM of DS18B20
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);

	return 1;
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_Is(uint8_t *ROM)
{
	if (*ROM == DS18B20_FAMILY_CODE) return 1; else return 0;
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_SetAlarmLowTemperature(OneWire_t *OneWire, uint8_t *ROM, int8_t temp)
{
	if (!DS18B20_Is(ROM)) return 0;

	if (temp > 125) temp = 125;
	if (temp < -55) temp = -55;

	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Read scratchpad command by onewire protocol
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);

	// Ignore first 2 bytes
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);

	uint8_t th = OneWire_ReadByte(OneWire);
	uint8_t tl = OneWire_ReadByte(OneWire);
	uint8_t conf = OneWire_ReadByte(OneWire);

	tl = (uint8_t)temp;


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Write scratchpad command by onewire protocol, only th, tl and conf register can be written
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);

	// Write bytes
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Copy scratchpad to EEPROM of DS18B20
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);

	return 1;
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_SetAlarmHighTemperature(OneWire_t *OneWire, uint8_t *ROM, int8_t temp)
{
	if (!DS18B20_Is(ROM)) return 0;

	if (temp > 125) temp = 125;
	if (temp < -55) temp = -55;


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Read scratchpad command by onewire protocol
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);

	/* Ignore first 2 bytes */
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);

	uint8_t th = OneWire_ReadByte(OneWire);
	uint8_t tl = OneWire_ReadByte(OneWire);
	uint8_t conf = OneWire_ReadByte(OneWire);

	th = (uint8_t)temp;


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Write scratchpad command by onewire protocol, only th, tl and conf register can be written
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);

	// Write bytes
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Copy scratchpad to EEPROM of DS18B20
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);

	return 1;
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_DisableAlarmTemperature(OneWire_t *OneWire, uint8_t *ROM)
{
	if (!DS18B20_Is(ROM)) return 0;


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Read scratchpad command by onewire protocol
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);

	// Ignore first 2 bytes
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);

	uint8_t th   = OneWire_ReadByte(OneWire);
	uint8_t tl   = OneWire_ReadByte(OneWire);
	uint8_t conf = OneWire_ReadByte(OneWire);

	th = 125;
	tl = (uint8_t)-55;


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Write scratchpad command by onewire protocol, only th, tl and conf register can be written
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);

	// Write bytes
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);


	OneWire_Reset(OneWire);// Reset line
	OneWire_SelectWithPointer(OneWire, ROM);// Select ROM number
	// Copy scratchpad to EEPROM of DS18B20
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);

	return 1;
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_AlarmSearch(OneWire_t *OneWire)
{
	return OneWire_Search(OneWire, DS18B20_CMD_ALARMSEARCH);// Start alarm search
}
//----------------------------------------------------------------------------------
uint8_t DS18B20_AllDone(OneWire_t *OneWire)
{
	// If read bit is low, then device is not finished yet with calculation temperature
	return OneWire_ReadBit(OneWire);
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//*******************************************************************************************

#endif

