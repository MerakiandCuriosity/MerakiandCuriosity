#include "encoder.h"
#include "led.h"
uint32_t gpio_interrup;
extern int32_t Get_Encoder_countA,encoderA_cnt,Get_Encoder_countB,encoderB_cnt;

void GROUP1_IRQHandler(void)
{
	gpio_interrup = DL_GPIO_getEnabledInterruptStatus(GPIOA,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
	//encoderA
	if((gpio_interrup & ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
	{
		if(!DL_GPIO_readPins(GPIOA,ENCODERA_E1B_PIN))
		{
			Get_Encoder_countA--;
		}
		else
		{
			Get_Encoder_countA++;
		}
	}
	else if((gpio_interrup & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
	{
		if(!DL_GPIO_readPins(GPIOA,ENCODERA_E1A_PIN))
		{
			Get_Encoder_countA++;
		}
		else
		{
			Get_Encoder_countA--;
		}
	}
	//encoderB
	if((gpio_interrup & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
	{
		if(!DL_GPIO_readPins(GPIOA,ENCODERB_E2B_PIN))
		{
			Get_Encoder_countB--;
		}
		else
		{
			Get_Encoder_countB++;
		}
	}
	else if((gpio_interrup & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
	{
		if(!DL_GPIO_readPins(GPIOA,ENCODERB_E2A_PIN))
		{
			Get_Encoder_countB++;
		}
		else
		{
			Get_Encoder_countB--;
		}
	}
	DL_GPIO_clearInterruptStatus(GPIOA,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN|ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
}

