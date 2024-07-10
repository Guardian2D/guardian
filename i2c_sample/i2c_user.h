
#ifndef I2C_USER_H
#define I2C_USER_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#include <stdint.h>
//////////////////////////////////////////////////////////////////////////////////////////
int I2C6_Init(void);
uint8_t IMU_ReadReg(int fd, unsigned char slave_addr, unsigned char reg_addr);
int16_t IMU_Read2Regs(int fd, unsigned char slave_addr, unsigned char reg_addr);
int32_t IMU_Read3Regs(int fd, unsigned char slave_addr, unsigned char reg_addr);
void IMU_WriteReg(int fd, uint8_t slave_addr, uint8_t write_reg, uint8_t write_data);
void IMU_CheckReg(int fd, uint8_t slave_addr, uint8_t reg_addr, char *regName);
//////////////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif