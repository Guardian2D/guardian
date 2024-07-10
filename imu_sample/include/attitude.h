
#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdint.h>
//////////////////////////////////////////////////////////////////////////////////////////
struct Attitude_Matrix
{
	double q0;
	double q1;
	double q2;
	double q3;
	double q0_prev;
	double q1_prev;
	double q2_prev;
	double q3_prev;
	double Q0_2;
	double Q1_2;
	double Q2_2;
	double Q3_2;
	double q0q1;
	double q1q2;
	double q1q3;
	double q2q3;
	double q0q2;
	double q0q3;
};

struct DeltaTheta
{
	double dx;
	double dy;
	double dz;
};

struct Velocity
{
	double Vx;
	double Vy;
	double Vz;
	double Vx_Prev;
	double Vy_Prev;
	double Vz_Prev;
	double AXsubG;
	double AYsubG;
	double AZsubG;
};

struct Position
{
	double Px;
	double Py;
	double High;
	double Px_prev;
	double Py_prev;
	double High_prev;
};

extern double pitch, roll, yaw;
 
extern struct Attitude_Matrix Cbn;
extern struct Velocity IMUvel;
extern struct Position IMUpos;

void Quaternions_Init(void);

void Attitude_Algorithm(void);

void Velocity_Algorithm(void);

void Location_Algorithm(void);

uint8_t Velocity_detect(void);

//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
