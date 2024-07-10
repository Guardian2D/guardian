
#ifndef GNSS_USER_H
#define GNSS_USER_H

#include <stdint.h>

#define LongSemiAxis  6378137.0
#define ShortSemiAxis  6356752.3142
#define Eccentricity_2  0.00669437999

struct GNSSdata
{
	int day_month_year;
	uint8_t hour, minute, second, day, month, year, GnssPosValid, GnssTimeValid;
	float hour_minute_second, Ground_speed, Course;
	double Latitude, Longitude, Height;
	double ENU_x, ENU_y, ENU_z;
	double ECEF_X, ECEF_Y, ECEF_Z;
	double ECEFinitX, ECEFinitY, ECEFinitZ;
};

extern char uart1ReadBuff[512];
extern uint8_t rcv1_flag;
extern struct GNSSdata GNSS;

uint8_t Uart1Init(void);
uint8_t Uart1Read();
uint8_t GNSS_Init(void);
uint8_t GNSS_GetData(void);
void Uart1Close();

#endif
