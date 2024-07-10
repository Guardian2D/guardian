
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include "imu_user.h"
#include "lhpf.h"
#include "attitude.h"
#include "kalman.h"
#include "gnss_user.h"

/*Global Variable*/
struct Attitude_Matrix Cbn={0};
struct Velocity IMUvel = {0};
struct Position IMUpos = {0};
double ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0, mx=0.0, my=0.0, mz=0.0;
uint32_t dt;	//us
double deltaT;	//s
double T31, T32, T33, T12, T22, pitch, roll, yaw;


// uint16_t i=0;
// uint32_t delta_t=0;
// double AAX=0.0, AAY=0.0, AAZ =0.0, AMX=0.0, HeightZ=0.0,
// 		AHX=0.0, AHY=0.0, AMX1=0.0, AMY1=0.0, AMZ1=0.0;	//Average data
// double sinP, cosP, sinR, cosR, sinY, cosY, pitch, roll, yaw;

void Quaternions_Init(void)
{
	uint16_t i=0;
	uint32_t delta_t=0;
	double Qtemp, norm, AAX=0.0, AAY=0.0, AAZ =0.0, AMX=0.0, HeightZ=0.0,
		   AMY=0.0, AMZ=0.0, AHX=0.0, AHY=0.0, AMX1=0.0, AMY1=0.0, AMZ1=0.0;	//Average data
	double sinP, cosP, sinR, cosR, sinY, cosY, pitch, roll, yaw;
	GX.P = 1.0, GY.P = 1.0, GZ.P = 1.0, Height.P = 1.0;
	GX.X = 0.0, GY.X = 0.0, GZ.X = 0.0, Height.X = 0.0;

	for(i=0; i<50; i++)
	{
		HeightZ += BMP280_GetHigh();
		usleep(10000);
	}
	IMUpos.High_prev = HeightZ/50.0;
	GNSS.Height = IMUpos.High_prev;

	for(i=0; i<200; i++)
	{
		IMU_GetData(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &delta_t);
		AAX += ax;
		AAY += ay;
		AAZ += az;
		AMX += mx;
		AMY += my;
		AMZ += mz;
		// GNSS_GetData();
	}
	AAX = AAX/200.0;
	AAY = AAY/200.0;
	AAZ = AAZ/200.0;
	AMX = AMX/200.0;
	AMY = AMY/200.0;
	AMZ = AMZ/200.0;
	GNSS.ECEFinitX = GNSS.ECEF_X;
	GNSS.ECEFinitY = GNSS.ECEF_Y;
	GNSS.ECEFinitZ = GNSS.ECEF_Z;
	
	/*Calculate pitch and roll*/
	pitch = asin(AAY/9.79136);
	roll = atan2(-AAX,AAZ);
	
	sinP = sin(pitch); 
	cosP = cos(pitch);
	sinR = sin(roll); 
	cosR = cos(roll);
	
	/*Transform system: b to n*/
	AMX1 = cosR*AMX + sinR*AMZ;
	AMY1 = sinR*sinP*AMX + cosP*AMY - sinP*cosR*AMZ;
	AMZ1 = -cosP*sinR*AMY + sinP*AMY + cosP*cosR*AMZ;
	
	AHX = AMX1*cosR + AMZ1*sinR;
	AHY = AMX1*sinP*sinR + AMY1*cosP - AMZ1*sinP*cosR;

	yaw = atan2(AHX, AHY);

	sinY = sin(yaw); 
	cosY = cos(yaw);
	
	/*Calculate quaternions, Transform n to b*/
	Cbn.q0_prev = sqrt( 1 + cosR*cosY - sinR*sinY*sinP + cosP*cosY + cosR*cosP )/2;
	Qtemp = 4.0 * Cbn.q0_prev;
	Cbn.q1_prev = (sinP - sinR*sinY + cosR*cosY*sinP) / Qtemp;
	Cbn.q2_prev = (sinR*cosY + cosR*sinY*sinP + cosP*sinR) / Qtemp;
	Cbn.q3_prev = (cosR*sinY + sinR*cosY*sinP + sinY*cosP) / Qtemp;
	norm = sqrt(Cbn.q0_prev*Cbn.q0_prev + Cbn.q1_prev*Cbn.q1_prev 
			+ Cbn.q2_prev*Cbn.q2_prev + Cbn.q3_prev*Cbn.q3_prev);
	
	Cbn.q0_prev = Cbn.q0_prev/norm;
	Cbn.q1_prev = Cbn.q1_prev/norm;
	Cbn.q2_prev = Cbn.q2_prev/norm;                                                
	Cbn.q3_prev = Cbn.q3_prev/norm;
	
	/*print eular angle*/
	printf("Pitch: %.3f, Roll: %.3f, Yaw: %.3f\r\n", pitch*57.29578, roll*57.29578, yaw*57.29578);
	
}


/*Global Variable*/
double Quat_prev[4];		//Quaternions
double ex, ey, ez, vectorX, vectorY, vectorZ, wx, wy, hx, hy, hz, by, bz;
struct DeltaTheta dtheta={0};

void Attitude_Algorithm(void)
{
	double Qnorm, delta_theta, sin1_2delta, cos1_2delta, module_m, module_a, max, may, maz, mmx, mmy, mmz,
	       delta_x_theta_sin, delta_y_theta_sin, delta_z_theta_sin;
		
	IMU_GetData(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz, &dt);

	// pitch = asin(ay/9.79136);
	// roll = atan2(-ax,az);
	
	// sinP = sin(pitch); 
	// cosP = cos(pitch);
	// sinR = sin(roll); 
	// cosR = cos(roll);
	
	// /*Transform system: b to n*/
	// AMX1 = cosR*mx + sinR*mz;
	// AMY1 = sinR*sinP*mx + cosP*my - sinP*cosR*mz;
	// AMZ1 = -cosP*sinR*my + sinP*my + cosP*cosR*mz;
	
	// AHX = AMX1*cosR + AMZ1*sinR;
	// AHY = AMX1*sinP*sinR + AMY1*cosP - AMZ1*sinP*cosR;

	// yaw = atan2(AHX, AHY);

	// pitch = 57.3*pitch;
	// roll = 57.3*roll;
	// yaw = -57.3*yaw;

/*====================卡尔曼滤波 Kalman Filtering====================*/
	
	deltaT = dt * 1e-6;
	
	module_m = sqrt(mx*mx + my*my + mz*mz);
	module_a = sqrt(ax*ax + ay*ay + az*az);
	mmx = mx / module_m;
	mmy = my / module_m;
	mmz = mz / module_m;
	max = ax / module_a;
	may = ay / module_a;
	maz = az / module_a;
	module_m = 1.5;
	module_a = 1.5;
	mmx = mx / module_m;
	mmy = my / module_m;
	mmz = mz / module_m;
	max = ax / module_a;
	may = ay / module_a;
	maz = az / module_a;
	
	/*body system to navigation system*/
	hx = mmx*(Cbn.Q0_2 + Cbn.Q1_2 - Cbn.Q2_2 - Cbn.Q3_2) + 2*mmy*(Cbn.q1q2 - Cbn.q0q3) + 2*mmz*(Cbn.q1q3 + Cbn.q0q2);
	hy = 2*mmx*(Cbn.q1q2 + Cbn.q0q3) + mmy*(Cbn.Q0_2 - Cbn.Q1_2 + Cbn.Q2_2 - Cbn.Q3_2) + 2*mmz*(Cbn.q2q3 - Cbn.q0q1);
	hz = 2*mmx*(Cbn.q1q3 - Cbn.q0q2) + 2*mmy*(Cbn.q2q3 + Cbn.q0q1) + mmz*(Cbn.Q0_2 - Cbn.Q1_2 - Cbn.Q2_2 + Cbn.Q3_2);
	by = sqrt( (hx*hx) + (hy*hy) );
	bz = hz;
	/*navigation system to body system*/
	wx = 2*by*(Cbn.q1q2 + Cbn.q0q3) + 2*bz*(Cbn.q1q3 - Cbn.q0q2);
	wy = by*(Cbn.Q0_2 - Cbn.Q1_2 + Cbn.Q2_2 - Cbn.Q3_2) + 2*bz*(Cbn.q0q1 + Cbn.q2q3);

	vectorX = 2 * (Cbn.q1q3 - Cbn.q0q2);
	vectorY = 2 * (Cbn.q2q3 + Cbn.q0q1);
	vectorZ = (Cbn.Q0_2 - Cbn.Q1_2 - Cbn.Q2_2 + Cbn.Q3_2);

	ex = may * vectorZ - maz * vectorY;
	ey = maz * vectorX - max * vectorZ;
	ez = mmx * wy - mmy * wx;	//max * vectorY - may * vectorX + 
	
	gx = kalman_filter(&GX, gx, ex, 0.1, 0.5);
	gy = kalman_filter(&GY, gy, ey, 0.1, 0.5);
	gz = kalman_filter(&GZ, gz, ez, 0.1, 0.5);
	
	//printf("x: %.5f, y: %.5f, z: %.5f\r\n", GX.K, GY.K, GZ.K);

	dtheta.dx = deltaT * gx;		//calculate delta angle
	dtheta.dy = deltaT * gy;
	dtheta.dz = deltaT * gz;

// //====================================开始解算！ Calculate Start!=========================================//
	
	dtheta.dx = dtheta.dx/ 57.2957805;
	dtheta.dy = dtheta.dy/ 57.2957805;
	dtheta.dz = dtheta.dz/ 57.2957805;
	
	delta_theta = sqrt( (dtheta.dx * dtheta.dx) + (dtheta.dy * dtheta.dy) + (dtheta.dz * dtheta.dz) );
	sin1_2delta = sin(delta_theta / 2.0), cos1_2delta = cos(delta_theta / 2.0);
	delta_x_theta_sin = (dtheta.dx / delta_theta) * sin1_2delta;
	delta_y_theta_sin = (dtheta.dy / delta_theta) * sin1_2delta;
	delta_z_theta_sin = (dtheta.dz / delta_theta) * sin1_2delta;
	
	/*Update quaternions*/
	Cbn.q0 = cos1_2delta * Cbn.q0_prev - delta_x_theta_sin * Cbn.q1_prev
	
			- delta_y_theta_sin * Cbn.q2_prev - delta_z_theta_sin * Cbn.q3_prev;

	Cbn.q1 = delta_x_theta_sin * Cbn.q0_prev + cos1_2delta * Cbn.q1_prev
			
			+ delta_z_theta_sin * Cbn.q2_prev - delta_y_theta_sin * Cbn.q3_prev;

	Cbn.q2 = delta_y_theta_sin * Cbn.q0_prev - delta_z_theta_sin * Cbn.q1_prev
			
			+ cos1_2delta * Cbn.q2_prev + delta_x_theta_sin * Cbn.q3_prev;

	Cbn.q3 = delta_z_theta_sin * Cbn.q0_prev + delta_y_theta_sin * Cbn.q1_prev
			
			- delta_x_theta_sin * Cbn.q2_prev + cos1_2delta * Cbn.q3_prev;


	Cbn.Q0_2 = Cbn.q0 * Cbn.q0;
	Cbn.Q1_2 = Cbn.q1 * Cbn.q1;
	Cbn.Q2_2 = Cbn.q2 * Cbn.q2;
	Cbn.Q3_2 = Cbn.q3 * Cbn.q3;
	
	Qnorm = sqrt(Cbn.Q0_2 + Cbn.Q1_2 + Cbn.Q2_2 + Cbn.Q3_2);
	
	if(Qnorm < 0.99997 || Qnorm > 1.00003)		//normalize quaternions
	{
		Cbn.q0 = Cbn.q0/Qnorm;
		Cbn.q1 = Cbn.q1/Qnorm;
		Cbn.q2 = Cbn.q2/Qnorm;                                                
		Cbn.q3 = Cbn.q3/Qnorm;
	}

	Cbn.q0q1 = Cbn.q0 * Cbn.q1;
	Cbn.q1q2 = Cbn.q1 * Cbn.q2;
	Cbn.q1q3 = Cbn.q1 * Cbn.q3;
	Cbn.q2q3 = Cbn.q2 * Cbn.q3;
	Cbn.q0q2 = Cbn.q0 * Cbn.q2;
	Cbn.q0q3 = Cbn.q0 * Cbn.q3;
	
	Quat_prev[0] = Cbn.q0_prev;
	Quat_prev[1] = Cbn.q1_prev;
	Quat_prev[2] = Cbn.q2_prev;
	Quat_prev[3] = Cbn.q3_prev;
	
	Cbn.q0_prev = Cbn.q0;
	Cbn.q1_prev = Cbn.q1;
	Cbn.q2_prev = Cbn.q2;
	Cbn.q3_prev = Cbn.q3;

	T31 = 2 * (Cbn.q1 * Cbn.q3 - Cbn.q0 * Cbn.q2);
	T32 = 2 * (Cbn.q2 * Cbn.q3 + Cbn.q0 * Cbn.q1);
	T33 = Cbn.Q0_2 - Cbn.Q1_2 - Cbn.Q2_2 + Cbn.Q3_2;
	T12 = 2 * (Cbn.q1 * Cbn.q2 - Cbn.q0 * Cbn.q3);
	T22 = Cbn.Q0_2 - Cbn.Q1_2 + Cbn.Q2_2 - Cbn.Q3_2;

	pitch = asin(T32) * 57.296;
	roll = atan2(-T31, T33) * 57.296;
	yaw = -atan2(T12, T22) * 57.296;

//	printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", vectorX*9.79136, ax, vectorY*9.79136, ay, vectorZ*9.79136, az);
}


/*Global Variable*/
double ax_prev=0.0, ay_prev=0.0, az_prev=0.0, dAx=0.0, dAy=0.0, dAz=0.0;
uint8_t SampleCount = 0;

void Velocity_Algorithm(void)
{
	double dVx, dVy, dVz;
	Attitude_Algorithm();
	
	if(Velocity_detect())
	{	
		IMUvel.Vx = 0.0;
		IMUvel.Vy = 0.0;
		IMUvel.Vz = 0.0;
		
		ax_prev = 0.0;
		ay_prev = 0.0;
		az_prev = 0.0;
		
		IMUvel.AXsubG = 0.0;
		IMUvel.AYsubG = 0.0;
		IMUvel.AZsubG = 0.0;
	}
	else
	{
		/*Transform System*/
		IMUvel.AXsubG = LHPFoperator(&AXLH, ax*(Cbn.Q0_2 + Cbn.Q1_2 - Cbn.Q2_2 - Cbn.Q3_2) + 2*ay*(Cbn.q1q2 - Cbn.q0q3) + 2*az*(Cbn.q1q3 + Cbn.q0q2));
		IMUvel.AYsubG = LHPFoperator(&AYLH, 2*ax*(Cbn.q1q2 + Cbn.q0q3) + ay*(Cbn.Q0_2 - Cbn.Q1_2 + Cbn.Q2_2 - Cbn.Q3_2) + 2*az*(Cbn.q2q3 - Cbn.q0q1));
		IMUvel.AZsubG = LHPFoperator(&AZLH, 2*ax*(Cbn.q1q3 - Cbn.q0q2) + 2*ay*(Cbn.q2q3 + Cbn.q0q1) + az*(Cbn.Q0_2 - Cbn.Q1_2 - Cbn.Q2_2 + Cbn.Q3_2) - 9.79136);
		
		/*Get deltaV*/
		dVx = 0.5 * (IMUvel.AXsubG + ax_prev) * deltaT;
		dVy = 0.5 * (IMUvel.AYsubG + ay_prev) * deltaT;
		dVz = 0.5 * (IMUvel.AZsubG + az_prev) * deltaT;
		
		ax_prev = IMUvel.AXsubG;
		ay_prev = IMUvel.AYsubG;
		az_prev = IMUvel.AZsubG;
		
		/*Calculate and fix velocity*/
		IMUvel.Vx = dVx + IMUvel.Vx;
		IMUvel.Vy = dVy + IMUvel.Vy;
		IMUvel.Vz = dVz + IMUvel.Vz;
		
		if(SampleCount > 15)
		{

			if( fabs(IMUvel.AXsubG - dAx) < 0.2 )
			{
				IMUvel.Vx = 0.0;
			}
			
			if( fabs(IMUvel.AYsubG - dAy) < 0.2 )
			{
				IMUvel.Vy = 0.0;
			}
			
			if( fabs(IMUvel.AZsubG - dAz) < 0.2 )
			{
				IMUvel.Vz = 0.0;
			}
			
			dAx = IMUvel.AXsubG;
			dAy = IMUvel.AYsubG;
			dAx = IMUvel.AZsubG;
			
			SampleCount = 0;
		}
		SampleCount++;
		
	}
}


uint8_t posCycleCount = 0;
double posDeltaT = 0.0;
void Location_Algorithm(void)
{
	Velocity_Algorithm();
	
	posCycleCount++;
	posDeltaT += deltaT;

	if(posCycleCount >= 2)
	{
		posCycleCount = 0;
		if(GNSS.GnssPosValid == 1)		//gnss valid
		{
			IMUpos.Px = 0.5 * (IMUvel.Vx + IMUvel.Vx_Prev) * posDeltaT + IMUpos.Px_prev;
			IMUpos.Px_prev = IMUpos.Px;

			IMUpos.Py = 0.5 * (IMUvel.Vy + IMUvel.Vy_Prev) * posDeltaT + IMUpos.Py_prev;
			IMUpos.Py_prev = IMUpos.Py;

			IMUvel.Vz = kalman_filter_pos(&Height, IMUvel.Vz, BMP280_GetHigh() - IMUpos.High, 0.1, 3.0);
			IMUpos.High = (IMUvel.Vz + IMUvel.Vz_Prev) * posDeltaT + IMUpos.High_prev;
			IMUpos.High_prev = IMUpos.High;

			IMUvel.Vx_Prev = IMUvel.Vx;
			IMUvel.Vy_Prev = IMUvel.Vy;
			IMUvel.Vz_Prev = IMUvel.Vz;

			posDeltaT = 0;
		}
		else
		{
			IMUpos.Px = 0.5 * (IMUvel.Vx + IMUvel.Vx_Prev) * deltaT + IMUpos.Px_prev;
			IMUpos.Px_prev = IMUpos.Px;

			IMUpos.Py = 0.5 * (IMUvel.Vy + IMUvel.Vy_Prev) * deltaT + IMUpos.Py_prev;
			IMUpos.Py_prev = IMUpos.Py;

			IMUvel.Vz = kalman_filter_pos(&Height, IMUvel.Vz, BMP280_GetHigh() - IMUpos.High, 0.1, 5.0);
			IMUpos.High = (IMUvel.Vz + IMUvel.Vz_Prev) * deltaT + IMUpos.High_prev;
			IMUpos.High_prev = IMUpos.High;

			IMUvel.Vx_Prev = IMUvel.Vx;
			IMUvel.Vy_Prev = IMUvel.Vy;
			IMUvel.Vz_Prev = IMUvel.Vz;
			
			posDeltaT = 0;
		}
	}
	/*TEST*/
	// IMUpos.Px = mx;
	// IMUpos.Py = my;
	// IMUpos.High = mz;
}

uint8_t Velocity_detect(void)
{
	double dCbn = fabs(Cbn.q0 - Quat_prev[0]) + fabs(Cbn.q1 - Quat_prev[1]) +
		   fabs(Cbn.q2 - Quat_prev[2]) + fabs(Cbn.q3 - Quat_prev[3]);
	
	if(dCbn < 0.00005)
	{
		return 1;	//static
	}
	
	return 0;	//runing
}

