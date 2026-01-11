#ifndef SSD_ARRAY_H
#define SSD_ARRAY_H

#include "stm32f4xx.h"

void SSD_init(void);
void SSD_update(int digitSelect, int value, int decimalPoint);

#endif // SSD_ARRAY_H