#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

// 原有函数声明
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Set_PWM(int pwma,int pwmb);

// 距离控制函数声明
void Move_Forward_Distance(float distance_cm);
void Move_Forward_20cm(void);
void Stop_Motors(void);
void Reset_Distance_Counter(void);
float Get_Current_Distance(void);

// 新增校准函数
void calibrate_motors(void);
void Move_Forward_Distance_Straight(float distance_cm);

// 宏定义
#define WHEEL_DIAMETER 6.5f
#define WHEEL_CIRCUMFERENCE 20.42f
#define ENCODER_PULSES_PER_REV 1000
#define TARGET_SPEED 300

#endif