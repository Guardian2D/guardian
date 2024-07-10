
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include "i2c_user.h"

int I2C6_Init(void)
{
    int fd;
    fd = open("/dev/i2c-6", O_RDWR);
    if(fd < 0){
        printf("Unable to open i2c-6 controlfile\n");
        return -1;
    }
    // if (ioctl(fd, I2C_SLAVE, 0x20) < 0) {
    //     printf("Failed to acquire bus access and/or talk to slave.\n");
    //     return 1;
    // }
    return fd;
}

void IMU_WriteReg(int fd, uint8_t slave_addr, uint8_t write_reg, uint8_t write_data)
{
    struct i2c_rdwr_ioctl_data i2c_write;
    int ret;
    uint8_t data[2];
    data[0] = write_reg;
    data[1] = write_data;
    struct i2c_msg msg;

    msg.addr = slave_addr;
    msg.flags = 0;
    msg.buf = data;
    msg.len = 2;

    i2c_write.msgs = &msg;
    i2c_write.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &i2c_write);
    if (ret < 0)
    {
        printf("write is error\n");
    }
}

uint8_t IMU_ReadReg(int fd, uint8_t slave_addr, uint8_t reg_addr)
{
    struct i2c_rdwr_ioctl_data i2c_read;
    uint8_t data;
    int ret;
    
    struct i2c_msg msg[2] = {
        [0]=  {
            .addr = slave_addr,
            .flags = 0, //write
            .buf = &reg_addr, //要发送的数据, 寄存器地址
            .len = 1 //size
        },
        [1]=  {
            .addr = slave_addr,
            .flags = 1, //read
            .buf = &data,
            .len = 1 //size
        },
    };
    i2c_read.msgs = msg;
    i2c_read.nmsgs = 2;
    
    ret = ioctl(fd, I2C_RDWR, &i2c_read);
    if (ret < 0)
    {
        printf("read is error\n");
        return -1;
    }
    // printf("ioctl is ok\n");
    return data;
}

int16_t IMU_Read2Regs(int fd, uint8_t slave_addr, uint8_t reg_addr)
{
    struct i2c_rdwr_ioctl_data i2c_read;
    uint8_t data[2];
    int ret;
    
    struct i2c_msg msg[2] = {
        [0] = {
            .addr = slave_addr,
            .flags = 0,
            .buf = &reg_addr,
            .len = 1
        },
        [1] = {
            .addr = slave_addr,
            .flags = 1,
            .buf = data,
            .len = 2
        }
    };
    i2c_read.msgs = msg;
    i2c_read.nmsgs = 2;
    
    ret = ioctl(fd, I2C_RDWR, &i2c_read);
    if (ret < 0)
    {
        printf("read is error\n");
        return -1;
    }
    // printf("ioctl is ok\n");
    return ((int16_t)((uint16_t)data[0])<<8 | (uint16_t)data[1]);
}

int32_t IMU_Read3Regs(int fd, uint8_t slave_addr, uint8_t reg_addr)
{
    struct i2c_rdwr_ioctl_data i2c_read;
    uint8_t data[3];
    int ret;
    
    struct i2c_msg msg[2] = {
        [0] = {
            .addr = slave_addr,
            .flags = 0,
            .buf = &reg_addr,
            .len = 1
        },
        [1] = {
            .addr = slave_addr,
            .flags = 1,
            .buf = data,
            .len = 3
        }
    };
    i2c_read.msgs = msg;
    i2c_read.nmsgs = 2;
    
    ret = ioctl(fd, I2C_RDWR, &i2c_read);
    if (ret < 0)
    {
        printf("read is error\n");
        return -1;
    }
    // printf("ioctl is ok\n");
    return ((uint32_t)data[0]<<12) | ((uint32_t)data[1]<<4) | ((uint32_t)data[2]>>4);
}

void IMU_CheckReg(int fd, uint8_t slave_addr, uint8_t reg_addr, char *regName)
{
	uint8_t data;
	data = IMU_ReadReg(fd, slave_addr, reg_addr);
	printf("REG %s's VALUE:0x%x \r\n", regName, data);
}


// int i2cFd = 0;
// int main(void)
// {
//     uint8_t data;
//     int16_t dat;
//     i2cFd = IMU_Init();

//     while(1)
//     {
//         data = IMU_ReadReg(i2cFd, 0x68, 0x75);
//         printf("%d\r\n", data);
//         sleep(1);

//         IMU_WriteReg(i2cFd, 0x68,  0x1C, 0x00);
//         data = IMU_ReadReg(i2cFd, 0x68, 0x1C);
//         printf("%d\r\n", data);
//         sleep(1);

//         IMU_WriteReg(i2cFd, 0x68,  0x1C, 0x08);
//         data = IMU_ReadReg(i2cFd, 0x68, 0x1C);
//         printf("%d\r\n", data);
//         sleep(1);

//         dat = IMU_Read2Regs(i2cFd, 0x68, 0x3F);
//         printf("%.3f\r\n", dat/1670.65);
//         sleep(1);

//         data = IMU_ReadReg(i2cFd, 0x68, 0x3F);
//         printf("%x\r\n", data);
//         sleep(1);
//     }
    
//     return 0;
// }





