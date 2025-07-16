#include "motor.h"

// Separate PID parameters for each motor
PID_Params_t Motor_A_PID = {1.00, 0.5, 0.0}; // Initial values same as original
PID_Params_t Motor_B_PID = {1.00, 0.5, 0.0}; // Can be tuned independently

// Debug variables to store last values
extern int Get_Encoder_countA, encoderA_cnt, Get_Encoder_countB, encoderB_cnt, PWMA, PWMB;

// Legacy global variables maintained for compatibility
float Velcity_Kp=1.00,  Velcity_Ki=0.5,  Velcity_Kd; //����ٶ�PID����
/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2024-07-019

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2024-07-019

All rights reserved
***********************************************/
/***************************************************************************
函数功能：电机PID死循环赋值
入口参数：目标速度，当前车轮的编码器值
返回  值：电机PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //定义相关变量
		static int ControlVelocityA, Last_biasA; //静态变量，函数调用结束后其值然存在
		
		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
		
		ControlVelocityA+=Motor_A_PID.Ki*(Bias-Last_biasA)+Motor_A_PID.Kp*Bias;  //增量式PI控制器
                                                                   //Motor_A_PID.Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Motor_A_PID.Ki*Bias             速度控制值由Bias不断积分得到 偏差越大速度越快
		Last_biasA=Bias;	
	    if(ControlVelocityA>3600) ControlVelocityA=3600;
	    else if(ControlVelocityA<-3600) ControlVelocityA=-3600;
		return ControlVelocityA; //返回速度控制值
}

/***************************************************************************
函数功能：电机PID死循环赋值
入口参数：目标速度，当前车轮的编码器值
返回  值：电机PWM
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //定义相关变量
		static int ControlVelocityB, Last_biasB; //静态变量，函数调用结束后其值然存在
		
		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
		
		ControlVelocityB+=Motor_B_PID.Ki*(Bias-Last_biasB)+Motor_B_PID.Kp*Bias;  //增量式PI控制器
                                                                   //Motor_B_PID.Kp*(Bias-Last_bias) 作用为限制加速度
	                                                                 //Motor_B_PID.Ki*Bias             速度控制值由Bias不断积分得到 偏差越大速度越快
		Last_biasB=Bias;	
	    if(ControlVelocityB>3600) ControlVelocityB=3600;
	    else if(ControlVelocityB<-3600) ControlVelocityB=-3600;
		return ControlVelocityB; //返回速度控制值
}

void Set_PWM(int pwma,int pwmb)
{
	//使用正点原子D153C电机模块时：PA12是AIN2，PA13是AIN1,PB16是BIN2，PB0是BIN1
	if(pwma>0)
	{
		DL_GPIO_setPins(AIN1_PORT,AIN1_PIN_12_PIN);
		DL_GPIO_clearPins(AIN2_PORT,AIN2_PIN_13_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwma),GPIO_PWM_0_C0_IDX);
	}
	else
	{
		DL_GPIO_setPins(AIN2_PORT,AIN2_PIN_13_PIN);
		DL_GPIO_clearPins(AIN1_PORT,AIN1_PIN_12_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwma),GPIO_PWM_0_C0_IDX);
	}
	if(pwmb>0)
	{
		DL_GPIO_setPins(BIN1_PORT,BIN1_Pin_Bin1_PIN);
		DL_GPIO_clearPins(BIN2_PORT,BIN2_Pin_Bin2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmb),GPIO_PWM_0_C1_IDX);
	}
    else
	{
		DL_GPIO_setPins(BIN2_PORT,BIN2_Pin_Bin2_PIN);
		DL_GPIO_clearPins(BIN1_PORT,BIN1_Pin_Bin1_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,ABS(pwmb),GPIO_PWM_0_C1_IDX);
	}		
}

/***************************************************************************
函数功能：电机调试信息打印
入口参数：无
返回  值：无
说明：输出编码器值、PWM值和PID参数，用于实时调试
***************************************************************************/
void Motor_Debug_Print(void)
{
    printf("Motor Debug Info:\n\r");
    printf("EncoderA: %d, EncoderB: %d\n\r", encoderA_cnt, encoderB_cnt);
    printf("PWMA: %d, PWMB: %d\n\r", PWMA, PWMB);
    printf("Motor_A PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\r", 
           Motor_A_PID.Kp, Motor_A_PID.Ki, Motor_A_PID.Kd);
    printf("Motor_B PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\r", 
           Motor_B_PID.Kp, Motor_B_PID.Ki, Motor_B_PID.Kd);
    printf("====================\n\r");
}

/***************************************************************************
函数功能：电机A校准
入口参数：kp, ki, kd - 新的PID参数
返回  值：无
说明：运行时调整电机A的PID参数
***************************************************************************/
void Motor_Calibrate_A(float kp, float ki, float kd)
{
    Motor_A_PID.Kp = kp;
    Motor_A_PID.Ki = ki;
    Motor_A_PID.Kd = kd;
    printf("Motor A calibrated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\r", kp, ki, kd);
}

/***************************************************************************
函数功能：电机B校准
入口参数：kp, ki, kd - 新的PID参数
返回  值：无
说明：运行时调整电机B的PID参数
***************************************************************************/
void Motor_Calibrate_B(float kp, float ki, float kd)
{
    Motor_B_PID.Kp = kp;
    Motor_B_PID.Ki = ki;
    Motor_B_PID.Kd = kd;
    printf("Motor B calibrated: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\r", kp, ki, kd);
}

/***************************************************************************
函数功能：获取电机调试信息
入口参数：指向编码器和PWM值的指针
返回  值：无
说明：通过指针返回当前的编码器值和PWM值，便于其他模块调用
***************************************************************************/
void Motor_Get_Debug_Info(int *encoderA, int *encoderB, int *pwmA, int *pwmB)
{
    if(encoderA != NULL) *encoderA = encoderA_cnt;
    if(encoderB != NULL) *encoderB = encoderB_cnt;
    if(pwmA != NULL) *pwmA = PWMA;
    if(pwmB != NULL) *pwmB = PWMB;
}
