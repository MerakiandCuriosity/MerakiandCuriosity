#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

#define KEY GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)

enum{
    key_stateless = 0,
    single_click,
    double_click,
    long_click
};

void KEY_Init(void);
u8 KEY_Scan(u16 Frequency, u16 filter_times);

#endif /* __KEY_H */