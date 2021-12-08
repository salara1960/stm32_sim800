/*
 * mq135.c
 *
 *  Created on: Dec 8, 2021
 *      Author: alarm
 */
#include "hdr.h"


#include "main.h"
#include "mq135.h"

#ifdef SET_MQ135

uint8_t pinMQ;

//-----------------------------------------------------------------------------
//Get the correction factor to correct for temperature and humidity
//param[in] t  The ambient air temperature
//param[in] h  The relative humidity
//return The calculated correction factor
//
float MQ135_getCorrectionFactor(float t, float h)
{
  return CORA * t * t - CORB * t + CORC - (h - 33.0) * CORD;
}
//-----------------------------------------------------------------------------
//Get the resistance of the sensor, ie. the measurement value
//return The sensor resistance in kOhm
//
float MQ135_getResistance()
{
  int val = analogRead(pinMQ);

  return ((1023.0 / (float)val) * 5.0 - 1.0) * RLOAD;
}
//-----------------------------------------------------------------------------
//Get the resistance of the sensor, ie. the measurement value corrected for temp/hum
//param[in] t  The ambient air temperature
//param[in] h  The relative humidity
//return The corrected sensor resistance kOhm
//
float MQ135_getCorrectedResistance(float t, float h)
{
  return getResistance() / getCorrectionFactor(t, h);
}
//-----------------------------------------------------------------------------
//Get the ppm of CO2 sensed (assuming only CO2 in the air)
//return The ppm of CO2 in the air
//
float MQ135_getPPM()
{
  return PARA * pow((getResistance() / RZERO), -PARB);
}
//-----------------------------------------------------------------------------
//Get the ppm of CO2 sensed (assuming only CO2 in the air), corrected for temp/hum
//param[in] t  The ambient air temperature
//param[in] h  The relative humidity
//return The ppm of CO2 in the air
//
float MQ135_getCorrectedPPM(float t, float h)
{
  return PARA * pow((getCorrectedResistance(t, h) / RZERO), -PARB);
}
//-----------------------------------------------------------------------------
//Get the resistance RZero of the sensor for calibration purposes
//return The sensor resistance RZero in kOhm
//
float MQ135_getRZero()
{
  return getResistance() * pow((ATMOCO2 / PARA), (1.0 / PARB));
}
//-----------------------------------------------------------------------------
//Get the corrected resistance RZero of the sensor for calibration purposes
//param[in] t  The ambient air temperature
//param[in] h  The relative humidity
//return The corrected sensor resistance RZero in kOhm
//
float MQ135_getCorrectedRZero(float t, float h)
{
  return getCorrectedResistance(t, h) * pow((ATMOCO2 / PARA), (1.0 / PARB));
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------



#endif
