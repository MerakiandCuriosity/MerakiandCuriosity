#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"

// Function prototypes
int angle_to_pwm_270(float angle);
int angle_to_pwm_180(float angle);
float base_x(float ts, float x_point);
float base_y(float ts, float y_point);

// External variables
extern float Voltage;
extern float real_arm_angle;
extern float real_base_angle;
extern float step_base;
extern float step_arm;
extern const float ts_dis;

// Timer interrupt handler
void TIM2_IRQHandler(void);

#endif /* __CONTROL_H */