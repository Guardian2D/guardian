
#ifndef KALMAN_H
#define KALMAN_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdint.h>
//////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	double P;
	double X;
	double K;
}KF;

double kalman_filter(KF *OBJ, double gyro, double EZ, double Q, double R);
double kalman_filter_pos(KF *OBJ, double velocity, double EZ, double Q, double R);

extern KF GX, GY, GZ, Height;

//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
