
#ifndef GPIO_USER_H
#define GPIO_USER_H

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

//LED:GPIO2_3,GPIO3_4; KEY:GPIO0_1,GPIO0_2; GYRO:GPIO0_6; ACC&MAG:GPIO3_5.
//LED:GPIO19,GPIO28;   KEY:GPIO1,GPIO2;     GYRO:GPIO6;   ACC&MAG:GPIO29.

#define LED1        (uint16_t)19
#define LED2        (uint16_t)28
#define KEY1        (uint16_t)1
#define KEY2        (uint16_t)2
#define ACCMAG_RST  (uint16_t)6
#define GYRO_RST    (uint16_t)29


#include <stdint.h>

uint8_t GPIO_Init(void);

void LED1_ON(void);

void LED1_OFF(void);

void LED2_Blink(void);

void LED2_ON(void);

void LED2_OFF(void);

int GetKey1Value(void);

int GetKey2Value(void);

void IMU_Reset(void);

void InitUart2Pin(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif
