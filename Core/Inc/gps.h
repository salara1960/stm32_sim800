#ifndef __GPS_H__
#define __GPS_H__

#include "hdr.h"
#include "main.h"

#ifdef SET_GPS

#define MAX_NMEA_MSG 2//3//4
#define NMEA_TYPE_LEN 6

enum {
	ixGNGGA = 0,
	ixGNRMC,
	ixGNGLL,
	ixGNVTG
};

//  Структура с переменными для запоминания данных геолокации
#pragma pack(push,1)
typedef struct {
    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;
    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;
    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;
    //
    //unsigned char crc;
    // GLL
//    char gll_status;
    // VTG - Course over ground, ground speed
//    float course_t; // ground speed true
//    char course_t_unit;
//    float course_m; // magnetic
//    char course_m_unit;
//    char speed_k_unit;
//    float speed_km; // speek km/hr
//    char speed_km_unit;
} gps_t;
#pragma pack(pop)

gps_t GPS;


int gpsValidate(char *str);
float gpsToDec(float deg, char nsew);
bool gpsParse(char *str);
char *gpsPrint(char *str);
uint8_t hexToBin(char *sc);

#endif

#endif
