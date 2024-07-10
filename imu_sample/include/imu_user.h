
#ifndef IMU_USER_H
#define IMU_USER_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdint.h>
#include "attitude.h"
#include "kalman.h"
#include "lhpf.h"
//////////////////////////////////////////////////////////////////////////////////////////

uint8_t IMU_Config(void);

uint8_t IMU_Init(void);

void IMU_GetData(double *AccX, double *AccY, double *AccZ, 
	 double *GyroX, double *GyroY, double *GyroZ, double *MagX, double *MagY, double *MagZ, uint32_t *dt);

void Gyroscope_bias(void);

double Get_temperature(void);

uint8_t Get_GYRO_ID(void);

uint8_t Get_A_M_ID(void);

extern uint32_t CycleT;

///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////GYRO
#define GYRO_ADDRESS			0x20
#define	GYRO_WHO_AM_I			0x0C	//0xD7
#define	IMU_GYRO_CTRL_REG0		0x0D
//|7:6| 5 |4:3|  2  | 1:0 |
//|LPF|SPI|HPF|HPFEN|SCALE|		00000011
#define	IMU_GYRO_CTRL_REG1		0x13
//| 7 | 6 | 5 |4:2| 1 | 0 |		00000010
//| - |RST|S-T|ODR|S/A|S/R|		
#define	IMU_GYRO_CTRL_REG2		0x14
#define	TEMP_OUT1				0x12
#define	DR_STATUS				0x07
#define	GYRO_XOUT_H				0x01
#define	GYRO_XOUT_L				0x02
#define	GYRO_YOUT_H				0x03
#define	GYRO_YOUT_L				0x04
#define	GYRO_ZOUT_H				0x05
#define	GYRO_ZOUT_L				0x06
//////////////////////////////////////////ACC&MAG
#define A_M_ADDRESS          	0x1C
#define	A_M_WHO_AM_I			0x0D	//0xC7
#define	SYSMOD					0x0B
#define	TEMP_OUT2				0x51
#define	IMU_A_CTRL_REG1			0x2A
//|7:6|5:3| 2 | 1 | 0 |			00000101
//| - |ODR| N | R | M |		
#define	IMU_A_CTRL_REG2			0x2B
//| 7 | 6 | 5 | 4:3 | 2 |1:0|	00000000
//|S-T|RST| - |SLEEP| M |OSR|
#define IMU_XYZ_DATA_CFG		0x0E
//|7|6|5| 4 |3|2| 1:0 |
//|-|-|-|HPF|-|-|SCALE|			00010000
#define HP_FILTER_CUTOFF		0x0F
#define IMU_A_DR_STATUS       	0x00	//Read the fourth binary bit data
#define	ACCEL_XOUT_H			0x01
#define	ACCEL_XOUT_L			0x02
#define	ACCEL_YOUT_H			0x03
#define	ACCEL_YOUT_L			0x04
#define	ACCEL_ZOUT_H			0x05
#define	ACCEL_ZOUT_L			0x06
////////////////////////////////////
#define	IMU_M_CTRL_REG1			0x5B
//| 7 | 6 | 5 |4:2|1:0|			00000011
//|A-C|RST|OST|OSR|HMS|
#define	IMU_M_CTRL_REG2			0x5C
//|7|6| 5 | 4 | 3 |  2  |1:0|	00000000
//|-|-|F-R|OFF| - |M-RST|A-S|
#define IMU_M_DR_STATUS       	0x02	//Read the fourth binary bit data
#define	MAG_XOUT_H				0x33
#define	MAG_XOUT_L				0x34
#define	MAG_YOUT_H				0x35
#define	MAG_YOUT_L				0x36
#define	MAG_ZOUT_H				0x37
#define	MAG_ZOUT_L				0x38

#define	CMP_XOUT_H				0x39
#define	CMP_XOUT_L				0x3A
#define	CMP_YOUT_H				0x3B
#define	CMP_YOUT_L				0x3C
#define	CMP_ZOUT_H				0x3D
#define	CMP_ZOUT_L				0x3E

//////////////////////////////////////////////////////////////////////////////////////////
//BMP280

uint8_t BMP280_Init(void);

uint8_t Get_BMP280ID(void);

double BMP280_GetTemperature(void);

double BMP280_GetPressure(void);

double BMP280_GetHigh(void);

int32_t BMP280_Compensate_T(int32_t adc_T);

uint32_t BMP280_Compensate_P(int32_t adc_P);

void Get_Compensate_Value(void);

#define SAP     			101325		//Standard Atmospheric Pressure:1013.25 hPA (1hPa = 100Pa = 1mbar)
#define P_OFFSET			500
#define H_OFFSET			-40
#define T_OFFSET			-3

#define BMP280_ADDRESS   			0x76
#define BMP280_CHIPID_REG			0xD0	//0x58 is correct
#define BMP280_RESET_REG			0xE0
#define BMP280_CTRL_HUMI			0xF2
#define BMP280_STATUS_REG			0xF3
#define BMP280_CTRL_MEA_REG			0xF4
#define BMP280_CONFIG_REG			0xF5
#define BMP280_PRESSURE_H			0xF7
#define BMP280_PRESSURE_L			0xF8
#define BMP280_PRESSURE_XL			0xF9
#define BMP280_TEMPERATURE_H		0xFA
#define BMP280_TEMPERATURE_L		0xFB
#define BMP280_TEMPERATURE_XL		0xFC
#define BMP280_VERSION				0xD1

#define BMP280_DIG_T1_LSB                0x88
#define BMP280_DIG_T1_MSB                0x89
#define BMP280_DIG_T2_LSB                0x8A
#define BMP280_DIG_T2_MSB                0x8B
#define BMP280_DIG_T3_LSB                0x8C
#define BMP280_DIG_T3_MSB                0x8D

#define BMP280_DIG_P1_LSB                0x8E
#define BMP280_DIG_P1_MSB                0x8F
#define BMP280_DIG_P2_LSB                0x90
#define BMP280_DIG_P2_MSB                0x91
#define BMP280_DIG_P3_LSB                0x92
#define BMP280_DIG_P3_MSB                0x93
#define BMP280_DIG_P4_LSB                0x94
#define BMP280_DIG_P4_MSB                0x95
#define BMP280_DIG_P5_LSB                0x96
#define BMP280_DIG_P5_MSB                0x97
#define BMP280_DIG_P6_LSB                0x98
#define BMP280_DIG_P6_MSB                0x99
#define BMP280_DIG_P7_LSB                0x9A
#define BMP280_DIG_P7_MSB                0x9B
#define BMP280_DIG_P8_LSB                0x9C
#define BMP280_DIG_P8_MSB                0x9D
#define BMP280_DIG_P9_LSB                0x9E
#define BMP280_DIG_P9_MSB                0x9F


void Accelerometer_sample(void);

void Accelerometer_bias(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
