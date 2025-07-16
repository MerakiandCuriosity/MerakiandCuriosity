#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"

// PID parameter structure for individual motor control
typedef struct {
    float Kp;  // Proportional gain
    float Ki;  // Integral gain
    float Kd;  // Derivative gain (currently unused but available)
} PID_Params_t;

// Debugging and calibration functions
void Motor_Debug_Print(void);
void Motor_Calibrate_A(float kp, float ki, float kd);
void Motor_Calibrate_B(float kp, float ki, float kd);
void Motor_Get_Debug_Info(int *encoderA, int *encoderB, int *pwmA, int *pwmB);

// Original functions (maintained for compatibility)
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
void Set_PWM(int pwma,int pwmb);

#endif