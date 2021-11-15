
#include "hdr.h"


#include "main.h"
#include "gps.h"

#ifdef SET_GPS
//-----------------------------------------------------------------------------

gps_t GPS;

const char *nmea[] = {
		"$GNGGA",
		"$GNRMC"
		//"$GNGLL",
		//"$GNVTG"
};

//-----------------------------------------------------------------------------

int gpsValidate(char *str)
{
char check[3] = {0};
char calcCRCstr[3] = {0};
int i = 0;
int calcCRC = 0;

    if (str[i] != '$') return 0; else i++;

    int8_t j = -1;
    for (int8_t k = 0; k < MAX_NMEA_MSG; k++) {
    	if (strstr(str, nmea[k])) {
    		j = k;
    		break;
    	}
    }
    if (j == -1) return 0; else return 1;

    while ((str[i] != 0) && (str[i] != '*') && (i < 75)) calcCRC ^= str[i++];

    if (i >= 75) return 0;

    if (str[i] == '*') {
        check[0] = str[i + 1];
        check[1] = str[i + 2];
        sprintf(calcCRCstr,"%02X", calcCRC);

        return ((calcCRCstr[0] == check[0]) && (calcCRCstr[1] == check[1])) ? 1 : 0 ;
    }

    return 0;
}
//-----------------------------------------------------------------------------
float gpsToDec(float deg, char nsew)
{
    int degree = (int)(deg / 100);
    float minutes = deg - degree * 100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') decimal *= -1;

    return decimal;
}
//----------------------------------------------------------------------------- -u_scanf_float
bool gpsParse(char *str)
{
bool ret = false;
int8_t idx = -1;

	for (int8_t i = 0; i < MAX_NMEA_MSG; i++) {
		if (!strncmp(str, nmea[i], NMEA_TYPE_LEN)) {
			idx = i;
			break;
		}
	}

	if (idx == -1) return ret;

	switch (idx) {
		case ixGNGGA://$GNGGA,163522.000,5443.66276,N,02032.21629,E,1,06,3.1,-5.0,M,0.0,M,,*57
			if (sscanf(str, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c",
					&GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew,
					&GPS.lock, &GPS.satelites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units) >= 9) {
    					//GPS.dec_latitude  = gpsToDec(GPS.nmea_latitude,  GPS.ns);
    					//GPS.dec_longitude = gpsToDec(GPS.nmea_longitude, GPS.ew);
    					ret = true;
			}
		break;
		case ixGNRMC://$GNRMC,163525.000,A,5443.66274,N,02032.21655,E,0.00,168.22,151121,,,A,V*05
			if (sscanf(str, "$GNRMC,%f,%c,%f,%c,%f,%c,%f,%f,%d",
					&GPS.utc_time, &GPS.rmc_status, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew,
					&GPS.speed_k, &GPS.course_d, &GPS.date) >= 9) {
					GPS.dec_latitude  = gpsToDec(GPS.nmea_latitude,  GPS.ns);
					GPS.dec_longitude = gpsToDec(GPS.nmea_longitude, GPS.ew);
					ret = true;
			}
		break;
		/*case ixGNGLL:
			if (sscanf(str, "$GNGLL,%f,%c,%f,%c,%f,%c",
					&GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time, &GPS.gll_status) >= 1) ret = true;
		break;
		case ixGNVTG:
			if (sscanf(str, "$GNVTG,%f,%c,%f,%c,%f,%c,%f,%c",
					&GPS.course_t, &GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit,
					&GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km, &GPS.speed_km_unit) >= 1) ret = true;
		break;*/
	}

    return ret;
}
//-----------------------------------------------------------------------------
char *gpsPrint(char *str)
{
	if (str) {
#ifdef SET_FLOAT_PART
		s_float_t flo = {0,0};
		floatPart(GPS.utc_time, &flo);      sprintf(str, "time:%06lu", flo.cel);
		sprintf(str+strlen(str), " date:%06d", GPS.date);
		floatPart(GPS.dec_latitude, &flo);  sprintf(str+strlen(str), " lat:%lu.%lu", flo.cel, flo.dro);
		floatPart(GPS.dec_longitude, &flo); sprintf(str+strlen(str), " lon:%lu.%lu sat:%d", flo.cel, flo.dro, GPS.satelites);
		floatPart(GPS.msl_altitude, &flo);  sprintf(str+strlen(str), " alt:%lu.%01lu", flo.cel, flo.dro/100000);
		floatPart(GPS.speed_k, &flo);       sprintf(str+strlen(str), " spd:%lu.%02lu", flo.cel, flo.dro/10000);
		floatPart(GPS.course_d, &flo);      sprintf(str+strlen(str), " dir:%lu.%02lu", flo.cel, flo.dro/10000);
#else

		sprintf(str, "time:%f date:%d lat:%.4f long:%.4f sat:%d alt:%.2f speed:%.2f dir:%.2f",
				GPS.utc_time, GPS.date, GPS.dec_latitude, GPS.dec_longitude,
				GPS.satelites, GPS.msl_altitude, GPS.speed_km, GPS.course_m);
#endif
	}

	return str;
}
//-----------------------------------------------------------------------------

#endif

