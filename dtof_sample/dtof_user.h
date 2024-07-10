
#ifndef DTOF_USER_H
#define DTOF_USER_H

#include <stdint.h>

uint8_t dtof_Init(void);

float dtofDistance(uint16_t cx_min, uint16_t cy_min, uint16_t cx_max, uint16_t cy_max);

#endif
