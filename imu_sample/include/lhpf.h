
#ifndef LHPF_H
#define LHPF_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdint.h>
//////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	double Tf;
	double y_prev;
	uint32_t timestamp_prev;
}LPF;

extern LPF AXL, AYL, AZL, MXL, MYL, MZL;

void LPF_init(void);

double LPFoperator(LPF* OBJ, double x);


typedef struct
{
	double Tf; //2Pi = 6, 1/1+(2Pi*Tf*Ts)=beta
	double x_prev;
	double y_prev;
	uint32_t timestamp_prev;
}LHPF;

extern LHPF AXLH, AYLH, AZLH;;

void LHPF_init(void);

double LHPFoperator(LHPF* OBJ, double x);

//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
