#include "motor.h"
float Velcity_Kp=1.00,  Velcity_Ki=0.5,  Velcity_Kd; // 速度环 PID 参数

/***************************************************************************
函数功能：电机 A 速度 PI 控制
输入：目标速度、当前速度
输出：PWM 控制值
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  // 速度偏差
		static int ControlVelocityA, Last_biasA; // 控制量和上次偏差
		
		Bias=TargetVelocity-CurrentVelocity; // 计算偏差
		
		ControlVelocityA+=Velcity_Ki*(Bias-Last_biasA)+Velcity_Kp*Bias;  // PI 控制计算
		Last_biasA=Bias;	
	    if(ControlVelocityA>3600) ControlVelocityA=3600;
	    else if(ControlVelocityA<-3600) ControlVelocityA=-3600;
		return ControlVelocityA; // 返回控制值
}

/***************************************************************************
函数功能：电机 B 速度 PI 控制
输入：目标速度、当前速度
输出：PWM 控制值
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  // 速度偏差
		static int ControlVelocityB, Last_biasB; // 控制量和上次偏差
		
		Bias=TargetVelocity-CurrentVelocity; // 计算偏差
		
		ControlVelocityB+=Velcity_Ki*(Bias-Last_biasB)+Velcity_Kp*Bias;  // PI 控制计算
		Last_biasB=Bias;	
	    if(ControlVelocityB>3600) ControlVelocityB=3600;
	    else if(ControlVelocityB<-3600) ControlVelocityB=-3600;
		return ControlVelocityB; // 返回控制值
}

void Set_PWM(int pwma,int pwmb)
{
	// TB6612 接线控制正反转：PA12-AIN2，PA13-AIN1，PB16-BIN2，PB0-BIN1
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
