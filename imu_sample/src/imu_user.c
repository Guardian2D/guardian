#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "usermain.h"
#include "gpio_user.h"
#include "imu_user.h"
#include "i2c_user.h"
#include "lhpf.h"
#include "attitude.h"

int i2cFd = 0;
uint8_t IMU_Config(void)
{
	uint8_t GYROID = 0, AMID = 0;
	double Temp = 0.0;

    //IMU Initialization
	i2cFd = I2C6_Init();
	if(i2cFd < 0){
		return 1;
	}
	usleep(20000);
	if(BMP280_Init()){
		return 1;
	}

	IMU_Reset();
	//CONFIG GYRO
	IMU_WriteReg(i2cFd, GYRO_ADDRESS, IMU_GYRO_CTRL_REG0, 0x02);	//LPF DISABEL, HPF DISABEL, +-500DPS
	usleep(20000);
	IMU_WriteReg(i2cFd, GYRO_ADDRESS, IMU_GYRO_CTRL_REG1, 0x02);	//ODR800Hz, ACTIVE
	printf("GYRO RESET COMPLTE.\r\n");
	usleep(20000);

	//CONFIG ACC
	IMU_WriteReg(i2cFd, A_M_ADDRESS, IMU_A_CTRL_REG1, 0x05);	//ODR400Hz, ACTIVE, NORMAL MODE, DENOISE
	usleep(30000);
	IMU_WriteReg(i2cFd, A_M_ADDRESS, IMU_A_CTRL_REG2, 0x00);	//AUTO SLEEP DSIABEL, OSR×2, NO SELF TEST
	usleep(20000);

	//CONFIG MAG
	IMU_WriteReg(i2cFd, A_M_ADDRESS, IMU_M_CTRL_REG1, 0x03);	//NO AUTO CALIBERATE, HYBERID MODE
	usleep(20000);
	IMU_WriteReg(i2cFd, A_M_ADDRESS, IMU_M_CTRL_REG2, 0x00);	//ETC
	usleep(20000);
	IMU_WriteReg(i2cFd, A_M_ADDRESS, IMU_XYZ_DATA_CFG, 0x00);	//HPF DISABEL, +-2G
	usleep(20000);
	printf("ACC AND MAG RESET COMPLTE.\r\n");
	
	//CHECK IMU ID AND PRINT TEMPERATURE
	GYROID = Get_GYRO_ID();
	usleep(20000);
	AMID = Get_A_M_ID();
	usleep(20000);
	Temp = Get_temperature();
	usleep(20000);
	
	printf("Temperature: %.1f Degrees Celsius,\r\n"
	"GYRO ID: 0x%x,\r\nAM ID: 0x%x.\r\n", Temp, GYROID, AMID);
	
	if(GYROID != 0xD7)
	{
		printf("GYRO ID ERROR!\r\n");
		while(1);
		return 1;
	}
	if(AMID != 0xC7)
	{
		printf("AM ID ERROR!\r\n");
		while(1);
		return 1;
	}
	
	// IMU_CheckReg(i2cFd, A_M_ADDRESS, SYSMOD, "SYSMOD");
	return 0;
}


double xb = 0.0, yb = 0.0, zb = 0.0;
uint32_t Ts = 0, Ts_prev = 0;

void IMU_GetData(double *AccX, double *AccY, double *AccZ, 
	 double *GyroX, double *GyroY, double *GyroZ, double *MagX, double *MagY, double *MagZ, uint32_t *dt)
{
	double axx, ayy, azz, gxx, gyy, gzz, mxx, myy, mzz, 
		   AX=0.0, AY=0.0, AZ=0.0, GX=0.0, GY=0.0, GZ=0.0, MX=0.0, MY=0.0, MZ=0.0;
	uint8_t cycle = 0, Count = 2;

	for(cycle = 0; cycle < 4; cycle++)
	{
		while(IMU_ReadReg(i2cFd, GYRO_ADDRESS, DR_STATUS)==0){}

		if(Count >= 2)
		{
			//Zero offset: 0.266599, -0.358561, 0.527586
			//K: 1.030080, 0.987872, 1.003745
			axx = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, ACCEL_XOUT_H)/1674.27636 * 1.0271 - 0.2392;	//UNIT: M/S/S
			AX += LPFoperator(&AXL, axx);
			
			ayy = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, ACCEL_YOUT_H)/1674.27636 * 0.9879 + 0.3586;
			AY += LPFoperator(&AYL, ayy);
			
			azz = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, ACCEL_ZOUT_H)/1674.27636 * 1.0038 - 0.5276;
			AZ += LPFoperator(&AZL, azz);

			//Zero offset: -213.119907, -85.991389, -64.491019
			//K: -0.960443, -0.980291, -0.967022
			mxx = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5 + 213.119907;
			MX += LPFoperator(&MXL, mxx);
			
			myy = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5 + 85.991389;
			MY += LPFoperator(&MYL, myy);
			
			mzz = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5 + 64.491019;
			MZ += LPFoperator(&MZL, mzz);
			
			Count = 0;
		}
		Count++;

		gxx = (double)IMU_Read2Regs(i2cFd, GYRO_ADDRESS, GYRO_XOUT_H)/64.0;	//UNIT: DEGREE/S
		GX += gxx;
		
		gyy = (double)IMU_Read2Regs(i2cFd, GYRO_ADDRESS, GYRO_YOUT_H)/64.0;
		GY += gyy;
		
		gzz = (double)IMU_Read2Regs(i2cFd, GYRO_ADDRESS, GYRO_ZOUT_H)/64.0;
		GZ += gzz;
		usleep(2500);
	}
	//Gyroscope bias: x: 0.254143, y: 0.090195, z: 0.410551
	*AccX = AX/2.0;
	*AccY = AY/2.0;
	*AccZ = AZ/2.0;
	*MagX = MX/2.0;
	*MagY = MY/2.0;
	*MagZ = MZ/2.0;
	*GyroX = 0.0;
	*GyroY = 0.0;
	*GyroZ = 0.0;

	Ts = Now_Timestamp();		//Record timestamp
	*dt = Ts - Ts_prev;
	Ts_prev = Ts;

	// printf("Tc:%d\r\n", *dt);
	// printf("gx:%.3f,gy:%.3f,gz:%.3f,  ax:%.3f,ay:%.3f,az:%.3f,  mx:%.3f,my:%.3f,mz:%.3f\r\n", 
	// 		*GyroX,*GyroY,*GyroZ, *AccX,*AccY,*AccZ, *MagX,*MagY,*MagZ);
}


void Gyroscope_bias(void)		//Constant value correction
{
	uint32_t i=0;
	double GX, GY, GZ;

	for(i=0; i<100000; i++)
	{
		while(IMU_ReadReg(i2cFd, GYRO_ADDRESS, DR_STATUS)==0){}

		GX = -(double)IMU_Read2Regs(i2cFd, GYRO_ADDRESS, GYRO_XOUT_H)/64.0;
		
		GY = -(double)IMU_Read2Regs(i2cFd, GYRO_ADDRESS, GYRO_YOUT_H)/64.0;
		
		GZ = (double)IMU_Read2Regs(i2cFd, GYRO_ADDRESS, GYRO_ZOUT_H)/64.0;

		xb = xb + GX;
		yb = yb + GY;
		zb = zb + GZ;

		usleep(10);
	}
	xb = xb / 100000.0;
	yb = yb / 100000.0;
	zb = zb / 100000.0;
	
	printf("Gyroscope bias: x: %.6f, y: %.6f, z: %.6f\r\n", xb, yb, zb);
	Ts_prev = Now_Timestamp();
}

double Get_temperature(void)
{
	uint8_t Temp1, Temp2;
	Temp1 = IMU_ReadReg(i2cFd, GYRO_ADDRESS, TEMP_OUT1);
	Temp2 = IMU_ReadReg(i2cFd, A_M_ADDRESS, TEMP_OUT2);
	return (Temp1+Temp2)/2.0;
}

uint8_t Get_GYRO_ID(void)
{
	uint8_t ID;
	ID = IMU_ReadReg(i2cFd, GYRO_ADDRESS, GYRO_WHO_AM_I);
	return ID;
}

uint8_t Get_A_M_ID(void)
{
	uint8_t ID;
	ID = IMU_ReadReg(i2cFd, A_M_ADDRESS, A_M_WHO_AM_I);
	return ID;
}


/////////////////////////////////////////////////BMP280//////////////////////////////////////////////////
uint8_t BMP280_Init(void)
{
	uint8_t BMPID;
	double temp, pres;
	IMU_WriteReg(i2cFd, BMP280_ADDRESS, BMP280_RESET_REG, 0xB6);	//Reset
	usleep(50000);
	IMU_WriteReg(i2cFd, BMP280_ADDRESS, BMP280_CTRL_MEA_REG, 0xB7);	//Oversampling t2x, p16x, normal mode
	usleep(20000);
	IMU_WriteReg(i2cFd, BMP280_ADDRESS, BMP280_CONFIG_REG, 0x10);	//Standby 0.5ms, IIR 16x, IIC
	usleep(20000);
	
	//CHECK BMP280ID AND DATA
	BMPID = Get_BMP280ID();
	usleep(10000);
	Get_Compensate_Value();
	usleep(10000);
	temp = BMP280_GetTemperature();
	usleep(10000);
	pres = BMP280_GetPressure();
	
	printf("Temperature: %.1f Degrees Celsius, Pressure: %.1f Pa, MP280ID: 0x%x.\n", temp, pres, BMPID);
	
	if(BMPID != 0x58)
	{
		printf("BMP280 ID ERROR!\n");
		return 1;
	}
	return 0;
}

double BMP280_GetPressure(void)
{
	double Pres;
	Pres = BMP280_Compensate_P(IMU_Read3Regs(i2cFd, BMP280_ADDRESS, BMP280_PRESSURE_H));
	return Pres/256.0 + P_OFFSET;
}

double BMP280_GetTemperature(void)
{
	double Temp;
	Temp = BMP280_Compensate_T(IMU_Read3Regs(i2cFd, BMP280_ADDRESS, BMP280_TEMPERATURE_H));
	return Temp/100.0;
}


double BMP280_GetHigh(void)
{
	double P, HIGH;
	BMP280_GetTemperature();
	P = BMP280_GetPressure();
	HIGH = 44330.0 * (1 - pow((P/SAP), 0.190295)) + H_OFFSET;
	return HIGH;
}

uint8_t Get_BMP280ID(void)
{
	uint8_t ID;
	ID = IMU_ReadReg(i2cFd, BMP280_ADDRESS, BMP280_CHIPID_REG);
	return ID;
}

uint8_t Dig[24];
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t T_fine;
void Get_Compensate_Value(void)
{
	struct i2c_rdwr_ioctl_data i2c_read;
    uint8_t reg_addr = BMP280_DIG_T1_LSB;
    int ret = 0;

    struct i2c_msg msg[2] = {
        [0] = {
            .addr = BMP280_ADDRESS,
            .flags = 0,
            .buf = &reg_addr,
            .len = 1
        },
        [1] = {
            .addr = BMP280_ADDRESS,
            .flags = 1,
            .buf = Dig,
            .len = 24
        }
    };
    i2c_read.msgs = msg;
    i2c_read.nmsgs = 2;
    
    ret = ioctl(i2cFd, I2C_RDWR, &i2c_read);
    if (ret < 0)
    {
        printf("read is error\n");
        return;
    }
	
	dig_T1 = (uint16_t)Dig[1]<<8|Dig[0];
	dig_T2 = (int16_t)Dig[3]<<8|Dig[2];
	dig_T3 = (int16_t)Dig[5]<<8|Dig[4];
	
	dig_P1 = (uint16_t)Dig[7]<<8|Dig[6];
	dig_P2 = (int16_t)Dig[9]<<8|Dig[8];
	dig_P3 = (int16_t)Dig[11]<<8|Dig[10];
	dig_P4 = (int16_t)Dig[13]<<8|Dig[12];
	dig_P5 = (int16_t)Dig[15]<<8|Dig[14];
	dig_P6 = (int16_t)Dig[17]<<8|Dig[16];
	dig_P7 = (int16_t)Dig[19]<<8|Dig[18];
	dig_P8 = (int16_t)Dig[21]<<8|Dig[20];
	dig_P9 = (int16_t)Dig[23]<<8|Dig[22];
}


int32_t BMP280_Compensate_T(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	T_fine = var1 + var2;
	T = (T_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BMP280_Compensate_P(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)T_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}


/////////////////////////////////////////////////////////////////////////////////////////
uint8_t IMU_Init(void)
{
	uint8_t ret = 0;
	ret = IMU_Config();
	if(ret != 0){
		return 1;
	}
	LPF_init();
	// LHPF_init();
	// Quaternions_Init();
	// Ts_prev = Now_Timestamp();
	return 0;
}


int main_imu(void)
{
	GPIO_Init();
	IMU_Init();
	// Accelerometer_sample();
	while(1)
	{
		Location_Algorithm();
		usleep(1000);
		printf("%.1f, %.1f, %.1f\r\n", pitch, roll, yaw);
	}
	return 0;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////
double ax_up=0.0, ax_down=0.0, ay_up=0.0, ay_down=0.0, az_up=0.0, az_down=0.0, ax_offset, ay_offset, az_offset, ax_k, ay_k, az_k;
double ax_, ay_, az_;
void Accelerometer_sample(void)		//Constant value correction
{
	uint16_t i = 0;
	printf("Sampling start!\r\n");

	//az up
	printf("Press key0, az up.\r\n");
	for(i=0; i<400; i++)
	{	
		ax_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5;
		
		ay_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5;
			
		az_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5;
		
		usleep(3000);
		printf("%.3f, %.3f, %.3f\r\n", ax_, ay_, az_);
		az_up += az_;
	}
	az_up = az_up / 400.0;
	usleep(5000000);


	//az down
	printf("Press key0, az down.\r\n");
	for(i=0; i<400; i++)
	{	
		ax_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5;
		
		ay_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5;
			
		az_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5;
		
		usleep(3000);
		printf("%.3f, %.3f, %.3f\r\n", ax_, ay_, az_);
		az_down += az_;
	}
	az_down = az_down / 400.0;
	usleep(5000000);
	
	//ay up
	printf("Press key0, ay up.\r\n");
	for(i=0; i<400; i++)
	{	
		ax_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5;
		
		ay_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5;
			
		az_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5;
		
		usleep(3000);
		printf("%.3f, %.3f, %.3f\r\n", ax_, ay_, az_);
		ay_up += ay_;
	}
	ay_up = ay_up / 400.0;
	usleep(5000000);


	//ay down
	printf("Press key0, ay down.\r\n");
	for(i=0; i<400; i++)
	{	
		ax_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5;
		
		ay_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5;
			
		az_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5;
		
		usleep(3000);
		printf("%.3f, %.3f, %.3f\r\n", ax_, ay_, az_);
		ay_down += ay_;
	}
	ay_down = ay_down / 400.0;
	usleep(5000000);
	
	//ax up
	printf("Press key0, ax up.\r\n");
	for(i=0; i<400; i++)
	{	
		ax_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5;
		
		ay_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5;
			
		az_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5;
		
		usleep(3000);
		printf("%.3f, %.3f, %.3f\r\n", ax_, ay_, az_);
		ax_up += ax_;
	}
	ax_up = ax_up / 400.0;
	usleep(5000000);

	//ax down
	printf("Press key0, ax down.\r\n");
	for(i=0; i<400; i++)
	{	
		ax_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_XOUT_H)/13.5;
		
		ay_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_YOUT_H)/13.5;
			
		az_ = (double)IMU_Read2Regs(i2cFd, A_M_ADDRESS, MAG_ZOUT_H)/13.5;
		
		usleep(3000);
		printf("%.3f, %.3f, %.3f\r\n", ax_, ay_, az_);
		ax_down += ax_;
	}
	ax_down = ax_down / 400.0;
	usleep(5000000);
	printf("Sampling completed!\r\n");
	Accelerometer_bias();
}


void Accelerometer_bias(void)
{
	ax_offset = (ax_up+ax_down)/2.0;
	ay_offset = (ay_up+ay_down)/2.0;
	az_offset = (az_up+az_down)/2.0;
	ax_k = 36/((ax_up - ax_down)/2.0);
	ay_k = 36/((ay_up - ay_down)/2.0);
	az_k = 36/((az_up - az_down)/2.0);
	

	printf("Zero offset: %.6f, %.6f, %.6f\r\n", ax_offset, ay_offset, az_offset);

	printf("K: %.6f, %.6f, %.6f\r\n", ax_k, ay_k, az_k);
}

