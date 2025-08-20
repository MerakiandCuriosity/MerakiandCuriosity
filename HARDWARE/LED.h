#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4)))

void LED_Init(void);

#endif /* __LED_H */