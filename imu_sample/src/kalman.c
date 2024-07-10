
#include <stdint.h>
#include <stdint.h>
#include "kalman.h"

KF GX, GY, GZ, Height;

double kalman_filter(KF *OBJ, double gyro, double EZ, double Q, double R)
{
	//先验估计
	OBJ->X = OBJ->X / 2 + gyro / 2;
	//先验估计协方差
	OBJ->P = OBJ->P / 4 + Q;
	//修正估计
	OBJ->X = OBJ->X + OBJ->K * EZ;
	//更新卡尔曼增益
	OBJ->K = OBJ->P / ( OBJ->P + R );
	//更新后验估计协方差
	OBJ->P = ( 1 - OBJ->K ) * OBJ->P;
	
	return OBJ->X;
}

double kalman_filter_pos(KF *OBJ, double velocity, double EZ, double Q, double R)
{
	//先验估计
	OBJ->X = OBJ->X / 2 + velocity / 2;
	//先验估计协方差
	OBJ->P = OBJ->P / 4 + Q;
	//修正估计
	OBJ->X = OBJ->X + OBJ->K * EZ;
	//更新卡尔曼增益
	OBJ->K = OBJ->P / ( OBJ->P + R );
	//更新后验估计协方差
	OBJ->P = ( 1 - OBJ->K ) * OBJ->P;
	
	return OBJ->X;
}
