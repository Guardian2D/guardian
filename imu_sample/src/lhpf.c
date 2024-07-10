
#include <stdint.h>
#include <stdint.h>
#include "usermain.h"
#include "lhpf.h"

LPF AXL, AYL, AZL, MXL, MYL, MZL;
void LPF_init(void)
{
	AXL.Tf = 0.5;
	AXL.y_prev = 0.0;
	AXL.timestamp_prev = Now_Timestamp();
	
	AYL.Tf = 0.5;
	AYL.y_prev = 0.0;
	AYL.timestamp_prev = Now_Timestamp();
	
	AZL.Tf = 0.5;
	AZL.y_prev = 0.0;
	AZL.timestamp_prev = Now_Timestamp();
	///////////////////////////////
	MXL.Tf = 1.0;
	MXL.y_prev = 0.0;
	MXL.timestamp_prev = Now_Timestamp();
	
	MYL.Tf = 1.0;
	MYL.y_prev = 0.0;
	MYL.timestamp_prev = Now_Timestamp();
	
	MZL.Tf = 1.0;
	MZL.y_prev = 0.0;
	MZL.timestamp_prev = Now_Timestamp();
}


double LPFoperator(LPF* OBJ, double x)
{
	unsigned long timestamp;
	double alpha, y, dt;
	
	timestamp = Now_Timestamp();
	dt = (timestamp - OBJ->timestamp_prev) * 1e-3;	//ms
	OBJ->timestamp_prev = timestamp;
	
	if(dt > 6)
	{
		OBJ->y_prev = x;
		return x;
	}
	
	alpha = OBJ->Tf / (OBJ->Tf + dt);
	
	y = alpha * x + (1.0 - alpha) * OBJ->y_prev;
	
	OBJ->y_prev = y;
	
	return y;
}


//============================================================
LHPF AXLH, AYLH, AZLH;

void LHPF_init(void)
{
	AXLH.Tf = 1.0;
	AXLH.x_prev = 0.0;
	AXLH.y_prev = 0.0;
	AXLH.timestamp_prev = Now_Timestamp();
	
	AYLH.Tf = 1.0;
	AYLH.x_prev = 0.0;
	AYLH.y_prev = 0.0;
	AYLH.timestamp_prev = Now_Timestamp();
	
	AZLH.Tf = 1.0;
	AZLH.x_prev = 0.0;
	AZLH.y_prev = 0.0;
	AZLH.timestamp_prev = Now_Timestamp();
}

double LHPFoperator(LHPF* OBJ, double x)
{
	unsigned long timestamp;
	double dt, beta, y;
	
	timestamp = Now_Timestamp();
	dt = (timestamp - OBJ->timestamp_prev) * 1e-6;	//us
	OBJ->timestamp_prev = timestamp;

	beta = 1/(1 + dt*OBJ->Tf);
	
	y = (x - OBJ->x_prev + OBJ->y_prev) * beta;
	
	OBJ->x_prev = x;
	OBJ->y_prev = y;
	
	return y;
}




