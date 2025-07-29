# 激光笔绘图主要代码

我的工程里面主要有SYSTEM，HARDWARE，MiniBalance三个文件夹

SYSTEM里面有三个文件，

```c
delay.c如下：
#include "delay.h"

static volatile uint32_t systick_time = 0;      //ÏµÍ³Ê±»ù
static volatile uint32_t systick_reloadval = 0; //±£´æµÄÖØ×°ÔØÖµ,ÓëÓÃ»§ÅäÖÃÓÐ¹Ø
static volatile uint32_t max_delayus_val = 0;   //×î´óusÑÓ³ÙÊ±¼ä
static volatile uint32_t systick_intFreq = 0;   //systickÖÐ¶ÏÆµÂÊ

//·µ»ØÏµÍ³µ±Ç°Ê±¿Ì
uint32_t User_GetSysTickTime(void)
{
	return systick_time;
}

/*
SysTickÊ±ÖÓÔ´Ö»ÓÐÁ½¸ö£ºAHB/8 »ò AHB (AHBÎªÏµÍ³Ê±ÖÓ)
*/
void SysTick_Init(uint32_t Fre)
{
	//ÉèÖÃsystickÖÐ¶ÏÓÅÏÈ¼¶
	NVIC_SetPriority(SysTick_IRQn,0); //²ÎÊý2-priority£ºÖ»Ê¹ÓÃ×îºó4bit,ÓëÖÐ¶ÏÓÅÏÈ¼¶·Ö×éÏà¹Ø,ÀýÈçÖÐ¶ÏÓÅÏÈ¼¶·Ö×é1£¬Ôò¸ß1bit±íÊ¾ÇÀÕ¼,µÍ3bit±íÊ¾ÏìÓ¦
	
	//¼ÇÂ¼ÅäÖÃµÄÖÐ¶ÏÆµÂÊ
	systick_intFreq = Fre;
	
	//Ñ¡ÔñAHB/8×÷ÎªSystickÊ±ÖÓÔ´
	SysTick->CTRL &= ~(1<<2); 
	
	//ÅäÖÃÖØ×°ÔØÖµ
	SysTick->LOAD = (SystemCoreClock/8)/Fre - 1; //SystickÒç³öÆµÂÊ = SysTickÊ±ÖÓ/(ÖØ×°ÔØÖµ+1)
	
	//¼ÇÂ¼ÖØ×°ÔØÖµ,±ãÓÚÓÃÓÚÑÓ³Ùº¯ÊýÊ¹ÓÃ
	systick_reloadval = (SystemCoreClock/8)/Fre;
	
	//¼ÆËã×î´óusÑÓÊ±µÄÊ±¼ä
	max_delayus_val = 1000000 / Fre;
	
	//Çå¿Õµ±Ç°¼ÆÊýÖµ
	SysTick->VAL = 0;
	
	//¿ªÆôÖÐ¶Ï
	SysTick->CTRL |= 1<<1;
	
	//Æô¶¯SysTick¼ÆÊýÆ÷
	SysTick->CTRL |= 1<<0;
}

//SysTickÖÐ¶Ï»Øµ÷º¯Êý
void SysTick_Handler(void)
{
	systick_time++;
}

//usÑÓ³Ù
void delay_us(uint32_t us)
{
	//×î´óµÄusÑÓ³ÙÓëÖÐ¶ÏÆµÂÊÓÐ¹Ø
	if( us > max_delayus_val ) us = max_delayus_val;
	
	//µ¥Î»×ª»»
	us = us*(SysTickFreq/1000000);
	
	//ÓÃÓÚ±£´æÒÑ×ß¹ýµÄÊ±¼ä
	uint32_t runningtime = 0;
	
	//»ñµÃµ±Ç°Ê±¿ÌµÄ¼ÆÊýÖµ
	uint32_t InserTick = SysTick->VAL;
	
	//ÓÃÓÚË¢ÐÂÊµÊ±Ê±¼ä
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//µÈ´ýÑÓ³Ù
	while(1)
	{
		tick = SysTick->VAL;//Ë¢ÐÂµ±Ç°Ê±¿Ì¼ÆÊýÖµ
		
		if( tick > InserTick ) countflag = 1;//³öÏÖÒç³öÂÖÑ¯,ÔòÇÐ»»×ßÊ±µÄ¼ÆËã·½Ê½
		
		if( countflag ) runningtime = InserTick + systick_reloadval - tick;
		else runningtime = InserTick - tick;
		
		if( runningtime>=us ) break;
	}
}

//msÑÓ³Ù
void delay_ms(uint32_t ms)
{
	uint32_t tickstart = User_GetSysTickTime();
	uint32_t wait = ms * (systick_intFreq / 1000);
	
	//µÈ´ýÑÓ³ÙÍê³É
    while ((User_GetSysTickTime() - tickstart) < wait)
    {
       
    }
}
sys.c如下：
#include "sys.h" 
#include "stm32f10x.h"

//THUMBÖ¸Áî²»Ö§³Ö»ã±àÄÚÁª
//²ÉÓÃÈçÏÂ·½·¨ÊµÏÖÖ´ÐÐ»ã±àÖ¸ÁîWFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//¹Ø±ÕËùÓÐÖÐ¶Ï
__asm void INTX_DISABLE(void)
{
	CPSID I;		  
}
//¿ªÆôËùÓÐÖÐ¶Ï
__asm void INTX_ENABLE(void)
{
	CPSIE I;		  
}
//ÉèÖÃÕ»¶¥µØÖ·
//addr:Õ»¶¥µØÖ·
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

//JTAGÄ£Ê½ÉèÖÃ,ÓÃÓÚÉèÖÃJTAGµÄÄ£Ê½
//mode:jtag,swdÄ£Ê½ÉèÖÃ;00,È«Ê¹ÄÜ;01,Ê¹ÄÜSWD;10,È«¹Ø±Õ;	   
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00		  
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //¿ªÆô¸¨ÖúÊ±ÖÓ	   
	AFIO->MAPR&=0XF8FFFFFF; //Çå³ýMAPRµÄ[26:24]
	AFIO->MAPR|=temp;       //ÉèÖÃjtagÄ£Ê½
} 
uart.c如下：
#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "stm32f10x.h"

#include "control.h"

//////////////////////////////////////////////////////////////////
//¼ÓÈëÒÔÏÂ´úÂë,Ö§³Öprintfº¯Êý,¶ø²»ÐèÒªÑ¡Ôñuse MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//±ê×¼¿âÐèÒªµÄÖ§³Öº¯Êý                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//¶¨Òå_sys_exit()ÒÔ±ÜÃâÊ¹ÓÃ°ëÖ÷»úÄ£Ê½    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//ÖØ¶¨Òåfputcº¯Êý 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

uint8_t Calculate_BBC(const uint8_t* checkdata,uint16_t datalen)
{
	char bccval = 0;
	for(uint16_t i=0;i<datalen;i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}

//Èí¸´Î»½øBootLoaderÇøÓò
static void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//½ÓÊÜµ½ÉÏÎ»»úÇëÇóµÄ¸´Î»×Ö·û¡°reset¡±£¬Ö´ÐÐÈí¼þ¸´Î»
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//½øÐÐÈí¼þ¸´Î»£¬¸´Î»ºóÖ´ÐÐ BootLoader ³ÌÐò
		}
	}
}


//´®¿Ú·¢ËÍÓë½ÓÊÕÊý¾Ý¶¨Òå
ReportDataSend_t ReportSendpack = { 0 };//Êý¾Ý·¢ËÍ°ü
ReportDataRecv_t ReportRecvpack = { 0 };//Êý¾Ý½ÓÊÜ°ü

//·¢ËÍÊý¾Ý³¤¶ÈÓëÊý¾Ý°ü
static const uint16_t report_send_bufferLen = sizeof(ReportDataSend_t);
static uint8_t report_send_buffer[report_send_bufferLen] = { 0 };

//½ÓÊÕÊý¾Ý³¤¶ÈÓëÊý¾Ý°ü
static const uint16_t report_recv_bufferLen = sizeof(ReportDataRecv_t);
static uint8_t report_recv_buffer[report_recv_bufferLen] = { 0 };

//·¢ËÍÊý¾Ýµ½ROS¶Ë
void report_to_ros(void)
{
	ReportSendpack.head1 = 0x1F;
	ReportSendpack.head2 = 0xF1;
//	ReportSendpack.arrive_flag = target_reach_flag;//Ä¿±êÎ»ÖÃµ½´ï±êÖ¾Î»
//	ReportSendpack.control_step = PTZ_ControlStep;//µ±Ç°²½½øÖµ
//	ReportSendpack.NowAngle = PTZ_NowAngle;//µ±Ç°½Ç¶È
	
	//½«ÐèÒª·¢ËÍµÄÊý¾Ý¸³Öµ
	memcpy(report_send_buffer,&ReportSendpack,sizeof(ReportDataSend_t));
	
	//Ö´ÐÐBCCÐ£Ñé
	report_send_buffer[report_send_bufferLen-1] = Calculate_BBC(report_send_buffer,report_send_bufferLen-1);
	
	//Æô¶¯·¢ËÍ
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel4,report_send_bufferLen);
    DMA_Cmd(DMA1_Channel4, ENABLE);
}



void usart1_init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStrue;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE); 

    //ÅäÖÃ DMA1 Í¨µÀ 4£¨USART1_TX£©
    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)report_send_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = report_send_bufferLen;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÍâÉè¼Ä´æÆ÷²»×ÔÔö
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;         //ÄÚ´æ×ÔÔö
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//ÍâÉèÊý¾Ý¿í¶È
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        //ÄÚ´æÊý¾Ý¿í¶È
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;      //ÆÕÍ¨Ä£Ê½
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//ÓÅÏÈ¼¶
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;       //ÄÚ´æµ½ÄÚ´æ°áÔË
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_Cmd (DMA1_Channel4,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	NVIC_InitStrue.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=5;//ÇÀÕ¼ÓÅÏÈ¼¶
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStrue);
}

/**************************************************************************
º¯Êý¹¦ÄÜ£º´®¿Ú1ÖÐ¶Ï·þÎñº¯Êý
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
//extern void update_PTZTarget(ReportDataRecv_t* p);

void USART1_IRQHandler(void)
{
	static uint8_t last_recv;
	static uint8_t report_recv_count = 0;
	static uint8_t report_recv_statemachine = 0;
	
	uint8_t recv = 0;
	
    if(USART_GetITStatus(USART1, USART_IT_RXNE)) //½ÓÊÕµ½Êý¾Ý
    {
		recv = USART_ReceiveData(USART1);
		_System_Reset_(recv);
		
		//¼òµ¥×´Ì¬»ú
		enum{
			Wait_HEAD=0       ,//µÈ´ýÕýÊ½Êý¾Ý½×¶Î
			Wait_FrameOver   ,//µÈ´ýÒ»Ö¡Êý¾Ý½ÓÊÕÍê±Ï
		};
		
		//Êý¾Ý½ÓÊÕÁ÷³Ì
		switch( report_recv_statemachine )
		{
			case Wait_HEAD://µÈ´ýÖ¡Í·
			{
				if(recv==0xFA && last_recv==0xAF)
				{
					report_recv_buffer[0] = 0xAF;
					report_recv_buffer[1] = 0xFA;
					report_recv_count = 2;
					report_recv_statemachine = Wait_FrameOver;
				}
				break;
			}
			case Wait_FrameOver: //µÈ´ýÒ»Ö¡Êý¾Ý½áÊø
			{
				report_recv_buffer[report_recv_count++] = recv;
				if( report_recv_count == report_recv_bufferLen )
				{	
					//¼ì²éÐ£ÑéÎ»ÊÇ·ñÕýÈ·
					if(report_recv_buffer[report_recv_bufferLen-1] == Calculate_BBC(report_recv_buffer,report_recv_bufferLen-1))
					{
						//Êý¾Ý¸³Öµ½âÎö
						memcpy(&ReportRecvpack,report_recv_buffer,sizeof(ReportDataRecv_t));
						
						//ÔÆÌ¨Êý¾Ý¸üÐÂ
//						update_PTZTarget(&ReportRecvpack);
					}
					
					//µÈ´ýÏÂÒ»´ÎÊý¾Ý½ÓÊÕ
					report_recv_count = 0;
					report_recv_statemachine = Wait_HEAD;
				}
				break;
			}
		}
		last_recv = recv;
    }
}
```

接着是HARDWARE文件夹

```c
led.c如下：
#include "led.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
oled.c如下：
/***********************************************
¹«Ë¾£ºÂÖÈ¤¿Æ¼¼(¶«Ý¸)ÓÐÏÞ¹«Ë¾
Æ·ÅÆ£ºWHEELTEC
¹ÙÍø£ºwheeltec.net
ÌÔ±¦µêÆÌ£ºshop114407458.taobao.com 
ËÙÂôÍ¨: https://minibalance.aliexpress.com/store/4455017
°æ±¾£ºV1.0
ÐÞ¸ÄÊ±¼ä£º2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update£º2022-10-13

All rights reserved
***********************************************/
#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
	   
u8 OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //ÉèÖÃÒ³µØÖ·£¨0~7£©
		OLED_WR_Byte (0x00,OLED_CMD);      //ÉèÖÃÏÔÊ¾Î»ÖÃ¡ªÁÐµÍµØÖ·
		OLED_WR_Byte (0x10,OLED_CMD);      //ÉèÖÃÏÔÊ¾Î»ÖÃ¡ªÁÐ¸ßµØÖ·   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}

//ÏòOLEDÐ´ÈëÒ»¸ö×Ö½Ú¡£
//dat:ÒªÐ´ÈëµÄÊý¾Ý/ÃüÁî
//cmd:Êý¾Ý/ÃüÁî±êÖ¾ 0,±íÊ¾ÃüÁî;1,±íÊ¾Êý¾Ý;
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 

	  	  
//¿ªÆôOLEDÏÔÊ¾    
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCÃüÁî
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//¹Ø±ÕOLEDÏÔÊ¾     
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCÃüÁî
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//ÇåÆÁº¯Êý,ÇåÍêÆÁ,Õû¸öÆÁÄ»ÊÇºÚÉ«µÄ!ºÍÃ»µãÁÁÒ»Ñù!!!	  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//¸üÐÂÏÔÊ¾
}
//»­µã 
//x:0~127
//y:0~63
//t:1 Ìî³ä 0,Çå¿Õ				   
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//³¬³ö·¶Î§ÁË.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}

//ÔÚÖ¸¶¨Î»ÖÃÏÔÊ¾Ò»¸ö×Ö·û,°üÀ¨²¿·Ö×Ö·û
//x:0~127
//y:0~63
//mode:0,·´°×ÏÔÊ¾;1,Õý³£ÏÔÊ¾				 
//size:Ñ¡Ôñ×ÖÌå 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//µÃµ½Æ«ÒÆºóµÄÖµ				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //µ÷ÓÃ1206×ÖÌå
		else temp=oled_asc2_1608[chr][t];		 //µ÷ÓÃ1608×ÖÌå 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
//m^nº¯Êý
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//ÏÔÊ¾2¸öÊý×Ö
//x,y :Æðµã×ø±ê	 
//len :Êý×ÖµÄÎ»Êý
//size:×ÖÌå´óÐ¡
//mode:Ä£Ê½	0,Ìî³äÄ£Ê½;1,µþ¼ÓÄ£Ê½
//num:ÊýÖµ(0~4294967295);	 		  
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
//ÏÔÊ¾×Ö·û´®
//x,y:Æðµã×ø±ê  
//*p:×Ö·û´®ÆðÊ¼µØÖ·
//ÓÃ16×ÖÌå
void OLED_ShowString(u8 x,u8 y,const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	   

//¸¡µãÊý·Ö½â
static int* FenJie_float(const float fudian)
{
	static int tmp[3];
	float temp;
	temp = fudian;
	if(temp<0) temp = -temp,tmp[2]=-1;
	else tmp[2]=1;
	
	tmp[0] = (int)temp; 			//»ñÈ¡ÕûÊý²¿·Ö
	tmp[1] = (temp - tmp[0])*1000;  //Ð¡Êý²¿·Ö
	return tmp;
}

//º¯Êý¹¦ÄÜ£ºOLEDÏÔÊ¾¸¡µãÊý
//Èë¿Ú²ÎÊý£ºÐèÒªÏÔÊ¾µÄ¸¡µãÊý¡¢x×ø±ê¡¢y×ø±ê¡¢ÕûÊýÏÔÊ¾µÄ¸öÊý¡¢Ð¡ÊýÏÔÊ¾µÄ¸öÊý
//·µ»ØÖµ£ºÎÞ
void OLED_ShowFloat(u8 show_x,u8 show_y,const float needtoshow,u8 zs_num,u8 xs_num)
{
	static int* p;
	p = FenJie_float(needtoshow);
	//·ûºÅÎ»ÏÔÊ¾
	if(p[2]>0) OLED_ShowChar(show_x,show_y,'+',12,1);	 
	else OLED_ShowChar(show_x,show_y,'-',12,1); 		 
	//ÕûÊý²¿·Ö
	OLED_ShowNumber(show_x+8,show_y,p[0],zs_num,12); 
	//Ð¡Êýµã
	OLED_ShowChar(show_x+6+8*zs_num,show_y,'.',12,1);
	//Ð¡Êý²¿·Ö
	if(p[1]<100)//ÐèÒª²¹Áã
	{	
		if(xs_num==3)
		{
			OLED_ShowNumber(show_x+12+8*zs_num,show_y,0,1,12);	 //²¹µÚÒ»¸ö0
			if(p[1]>=10)		
				OLED_ShowNumber(show_x+18+8*zs_num,show_y,p[1],2,12);	 //ºóÁ½Î»²»ÐèÒª²¹0
			else
				OLED_ShowNumber(show_x+18+8*zs_num,show_y,0,1,12), //²¹µÚ¶þ¸ö0
				OLED_ShowNumber(show_x+24+8*zs_num,show_y,p[1],1,12);
		}
		else
		{
			if(p[1]>=0&&p[1]<100)
				OLED_ShowNumber(show_x+12+8*zs_num,show_y,0,1,12);	 //²¹µÚÒ»¸ö0
				OLED_ShowNumber(show_x+18+8*zs_num,show_y,p[1]/10,1,12);
		}
	}
	else
	{
		if(xs_num==3)
		{
			OLED_ShowNumber(show_x+12+8*zs_num,show_y,p[1],3,12);
		}	
		else
			OLED_ShowNumber(show_x+12+8*zs_num,show_y,p[1]/10,2,12);
	}
}



//³õÊ¼»¯OLED					    
void OLED_Init(void)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //Ê¹ÄÜPB¶Ë¿ÚÊ±ÖÓ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//¶Ë¿ÚÅäÖÃ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //2M
	GPIO_Init(GPIOB, &GPIO_InitStructure);					      //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIO 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//¿ªA¿ÚÊ±ÖÓ¡£
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;       //ÉèÎªÊä³ö¡¡
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	PWR_BackupAccessCmd(ENABLE);//ÔÊÐíÐÞ¸ÄRTC ºÍºó±¸¼Ä´æÆ÷
	RCC_LSEConfig(RCC_LSE_OFF);//¹Ø±ÕÍâ²¿µÍËÙÍâ²¿Ê±ÖÓÐÅºÅ¹¦ÄÜ ºó£¬PC13 PC14 PC15 ²Å¿ÉÒÔµ±ÆÕÍ¨IOÓÃ¡£
	BKP_TamperPinCmd(DISABLE);//¹Ø±ÕÈëÇÖ¼ì²â¹¦ÄÜ£¬Ò²¾ÍÊÇ PC13£¬Ò²¿ÉÒÔµ±ÆÕÍ¨IO Ê¹ÓÃ
	PWR_BackupAccessCmd(DISABLE);//½ûÖ¹ÐÞ¸Äºó±¸¼Ä´æÆ÷

	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD); //¹Ø±ÕÏÔÊ¾
	OLED_WR_Byte(0xD5,OLED_CMD); //ÉèÖÃÊ±ÖÓ·ÖÆµÒò×Ó,Õðµ´ÆµÂÊ
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],·ÖÆµÒò×Ó;[7:4],Õðµ´ÆµÂÊ
	OLED_WR_Byte(0xA8,OLED_CMD); //ÉèÖÃÇý¶¯Â·Êý
	OLED_WR_Byte(0X3F,OLED_CMD); //Ä¬ÈÏ0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //ÉèÖÃÏÔÊ¾Æ«ÒÆ
	OLED_WR_Byte(0X00,OLED_CMD); //Ä¬ÈÏÎª0

	OLED_WR_Byte(0x40,OLED_CMD); //ÉèÖÃÏÔÊ¾¿ªÊ¼ÐÐ [5:0],ÐÐÊý.
													    
	OLED_WR_Byte(0x8D,OLED_CMD); //µçºÉ±ÃÉèÖÃ
	OLED_WR_Byte(0x14,OLED_CMD); //bit2£¬¿ªÆô/¹Ø±Õ
	OLED_WR_Byte(0x20,OLED_CMD); //ÉèÖÃÄÚ´æµØÖ·Ä£Ê½
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00£¬ÁÐµØÖ·Ä£Ê½;01£¬ÐÐµØÖ·Ä£Ê½;10,Ò³µØÖ·Ä£Ê½;Ä¬ÈÏ10;
	OLED_WR_Byte(0xA1,OLED_CMD); //¶ÎÖØ¶¨ÒåÉèÖÃ,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //ÉèÖÃCOMÉ¨Ãè·½Ïò;bit3:0,ÆÕÍ¨Ä£Ê½;1,ÖØ¶¨ÒåÄ£Ê½ COM[N-1]->COM0;N:Çý¶¯Â·Êý
	OLED_WR_Byte(0xDA,OLED_CMD); //ÉèÖÃCOMÓ²¼þÒý½ÅÅäÖÃ
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]ÅäÖÃ
		 
	OLED_WR_Byte(0x81,OLED_CMD); //¶Ô±È¶ÈÉèÖÃ
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;Ä¬ÈÏ0X7F (ÁÁ¶ÈÉèÖÃ,Ô½´óÔ½ÁÁ)
	OLED_WR_Byte(0xD9,OLED_CMD); //ÉèÖÃÔ¤³äµçÖÜÆÚ
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //ÉèÖÃVCOMH µçÑ¹±¶ÂÊ
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //È«¾ÖÏÔÊ¾¿ªÆô;bit0:1,¿ªÆô;0,¹Ø±Õ;(°×ÆÁ/ºÚÆÁ)
	OLED_WR_Byte(0xA6,OLED_CMD); //ÉèÖÃÏÔÊ¾·½Ê½;bit0:1,·´ÏàÏÔÊ¾;0,Õý³£ÏÔÊ¾	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //¿ªÆôÏÔÊ¾	 
	OLED_Clear();
}  




servo.c如下：
#include "servo.h"

#include "stm32f10x.h"

void TIM4_PWM_Init(uint16_t arr,uint16_t psc)
{		 			
	 GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	                                                                     	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler =psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	
	TIM_Cmd(TIM4, ENABLE);//Ê¹ÄÜTIM4
	
	//ÖÐÖµ
	TIM4->CCR3 = PTZ_MIDDLE;
	TIM4->CCR4 = PTZ_MIDDLE;
} 

timer.c如下：
#include "timer.h"

void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //Ê±ÖÓÊ¹ÄÜ

    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler =psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM2, ENABLE);

}
adc.c如下：
/***********************************************
¹«Ë¾£ºÂÖÈ¤¿Æ¼¼(¶«Ý¸)ÓÐÏÞ¹«Ë¾
Æ·ÅÆ£ºWHEELTEC
¹ÙÍø£ºwheeltec.net
ÌÔ±¦µêÆÌ£ºshop114407458.taobao.com
ËÙÂôÍ¨: https://minibalance.aliexpress.com/store/4455017
°æ±¾£ºV1.0
ÐÞ¸ÄÊ±¼ä£º2022-10-13

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update£º2022-10-13

All rights reserved
***********************************************/
#include "adc.h"

/**************************************************************************
º¯Êý¹¦ÄÜ£ºACD³õÊ¼»¯µç³ØµçÑ¹¼ì²â
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void  Adc_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //Ê¹ÄÜADC1Í¨µÀÊ±ÖÓ
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //ÉèÖÃADC·ÖÆµÒò×Ó6 72M/6=12,ADC×î´óÊ±¼ä²»ÄÜ³¬¹ý14M
    //ÉèÖÃÄ£ÄâÍ¨µÀÊäÈëÒý½Å
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//Ä£ÄâÊäÈëÒý½Å
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    ADC_DeInit(ADC1);  //¸´Î»ADC1,½«ÍâÉè ADC1 µÄÈ«²¿¼Ä´æÆ÷ÖØÉèÎªÈ±Ê¡Öµ
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC¹¤×÷Ä£Ê½:ADC1ºÍADC2¹¤×÷ÔÚ¶ÀÁ¢Ä£Ê½
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//Ä£Êý×ª»»¹¤×÷ÔÚµ¥Í¨µÀÄ£Ê½
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//Ä£Êý×ª»»¹¤×÷ÔÚµ¥´Î×ª»»Ä£Ê½
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//×ª»»ÓÉÈí¼þ¶ø²»ÊÇÍâ²¿´¥·¢Æô¶¯
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADCÊý¾ÝÓÒ¶ÔÆë
    ADC_InitStructure.ADC_NbrOfChannel = 1;	//Ë³Ðò½øÐÐ¹æÔò×ª»»µÄADCÍ¨µÀµÄÊýÄ¿
    ADC_Init(ADC1, &ADC_InitStructure);	//¸ù¾ÝADC_InitStructÖÐÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯ÍâÉèADCxµÄ¼Ä´æÆ÷
    ADC_Cmd(ADC1, ENABLE);	//Ê¹ÄÜÖ¸¶¨µÄADC1
    ADC_ResetCalibration(ADC1);	//Ê¹ÄÜ¸´Î»Ð£×¼
    while(ADC_GetResetCalibrationStatus(ADC1));	//µÈ´ý¸´Î»Ð£×¼½áÊø
    ADC_StartCalibration(ADC1);	 //¿ªÆôADÐ£×¼
    while(ADC_GetCalibrationStatus(ADC1));	 //µÈ´ýÐ£×¼½áÊø
}

/**************************************************************************
º¯Êý¹¦ÄÜ£ºAD²ÉÑù
Èë¿Ú²ÎÊý£ºADC1 µÄÍ¨µÀ
·µ»Ø  Öµ£ºAD×ª»»½á¹û
**************************************************************************/
u16 Get_Adc(u8 ch)
{
    //ÉèÖÃÖ¸¶¨ADCµÄ¹æÔò×éÍ¨µÀ£¬Ò»¸öÐòÁÐ£¬²ÉÑùÊ±¼ä
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCÍ¨µÀ,²ÉÑùÊ±¼äÎª239.5ÖÜÆÚ
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//Ê¹ÄÜÖ¸¶¨µÄADC1µÄÈí¼þ×ª»»Æô¶¯¹¦ÄÜ
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//µÈ´ý×ª»»½áÊø
    return ADC_GetConversionValue(ADC1);	//·µ»Ø×î½üÒ»´ÎADC1¹æÔò×éµÄ×ª»»½á¹û
}


/**************************************************************************
º¯Êý¹¦ÄÜ£º¶ÁÈ¡µç³ØµçÑ¹
Èë¿Ú²ÎÊý£ºÎÞ
·µ»Ø  Öµ£ºµç³ØµçÑ¹ µ¥Î»MV
**************************************************************************/
int Get_battery_volt(void)
{
    int Volt;//µç³ØµçÑ¹
    Volt=Get_Adc(Battery_Ch)*3.3*11*100/4096;	//µç×è·ÖÑ¹£¬¾ßÌå¸ù¾ÝÔ­ÀíÍ¼¼òµ¥·ÖÎö¿ÉÒÔµÃµ½
    return Volt;
}


key.c如下：
#include "key.h"

void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //Ê¹ÄÜ¶Ë¿ÚÊ±ÖÓ
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_5;	//¶Ë¿ÚÅäÖÃ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //ÉÏÀ­ÊäÈë
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIO
} 


/**************************************************************************
º¯Êý¹¦ÄÜ£º°´¼üÉ¨Ãèº¯Êý
Èë¿Ú²ÎÊý£ºÖ´ÐÐ¸Ãº¯ÊýµÄÈÎÎñÆµÂÊ¡¢ÑÓ³ÙÂË²¨µÄÊ±¼ä
·µ»Ø  Öµ£ºlong_click¡¢double_click¡¢single_click¡¢key_stateless£¨³¤°´¡¢Ë«»÷¡¢µ¥»÷¡¢ÎÞ×´Ì¬£©
×÷    Õß£ºWHEELTEC
**************************************************************************/
u8 KEY_Scan(u16 Frequency,u16 filter_times)
{
    static u16 time_core;//×ßÊ±ºËÐÄ
    static u16 long_press_time;//³¤°´Ê¶±ð
    static u8 press_flag=0;//°´¼ü°´ÏÂ±ê¼Ç
    static u8 check_once=0;//ÊÇ·ñÒÑ¾­Ê¶±ð1´Î±ê¼Ç
    static u16 delay_mini_1;
    static u16 delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//Ëã³ö¼Æ1ÐèÒª¶àÉÙ¸öºÁÃë

    if(check_once)//Íê³ÉÁËÊ¶±ð£¬ÔòÇå¿ÕËùÓÐ±äÁ¿
    {
        press_flag=0;//Íê³ÉÁË1´ÎÊ¶±ð£¬±ê¼ÇÇåÁã
        time_core=0;//Íê³ÉÁË1´ÎÊ¶±ð£¬Ê±¼äÇåÁã
        long_press_time=0;//Íê³ÉÁË1´ÎÊ¶±ð£¬Ê±¼äÇåÁã
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY==1) check_once=0; //Íê³ÉÉ¨Ãèºó°´¼ü±»µ¯Æð£¬Ôò¿ªÆôÏÂÒ»´ÎÉ¨Ãè

    if(KEY==0&&check_once==0)//°´¼üÉ¨Ãè
    {
        press_flag=1;//±ê¼Ç±»°´ÏÂ1´Î
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(u16)(500.0f/Count_time))// ³¤°´1Ãë
    {	
        check_once=1;//±ê¼ÇÒÑ±»Ê¶±ð
        return long_click; //³¤°´
    }

    //°´¼ü±»°´ÏÂ1´ÎÓÖµ¯Æðºó£¬¿ªÆôÄÚºË×ßÊ±
    if(press_flag&&KEY==1)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(u16)(50.0f/Count_time)&&time_core<(u16)(300.0f/Count_time)))//50~700msÄÚ±»ÔÙ´Î°´ÏÂ
    {
        if(KEY==0) //Èç¹ûÔÙ´Î°´ÏÂ
        {
            check_once=1;//±ê¼ÇÒÑ±»Ê¶±ð
            return double_click; //±ê¼ÇÎªË«»÷
        }
    }
    else if(press_flag&&time_core>(u16)(300.0f/Count_time))
    {
        check_once=1;//±ê¼ÇÒÑ±»Ê¶±ð
        return single_click; //800msºó»¹Ã»±»°´ÏÂ£¬ÔòÊÇµ¥»÷
    }

    return key_stateless;
}

```

最后是MiniBalance文件夹：

```c
control.c如下：
#include <math.h>

#include "control.h"

#include "adc.h"
#include "key.h"
#include "usart.h"
#include "LED.h"

#include "stm32f10x.h"

/********************************************
*                                           *
*               ÄÚ²¿±äÁ¿                     *
*                                           *
********************************************/
//µç³ØµçÑ¹²É¼¯¸¨×é±äÁ¿
static int Voltage_All = 0;
static uint8_t Voltage_Count = 0;


/********************************************
*                                           *
*               È«¾Ö±äÁ¿                     *
*                                           *
********************************************/
float Voltage;//µç³ØµçÑ¹


// ½«½Ç¶È×ª»»ÎªPWMÖµ£¨ÊÊÓÃÓÚ270¶È¶æ»ú£©
int angle_to_pwm_270(float angle) {
    // ÏÞÖÆ½Ç¶ÈÔÚ0µ½270¶ÈÖ®¼ä
    if (angle < 0.0f) {
        angle = 0.0f;
    } else if (angle > 270.0f) {
        angle = 270.0f;
    }
    
    // ÏßÐÔ×ª»»¹«Ê½£ºPWM = (angle * 1000 / 270) + 250
    float pwm = (angle * 1000.0f / 270.0f) + 250.0f;
    return (int)roundf(pwm); // ËÄÉáÎåÈëµ½ÕûÊý
}

int angle_to_pwm_180(float angle) {

    if (angle < 0.0f) {
        angle = 0.0f;
    } else if (angle > 180.0f) {
        angle = 180.0f;
    }
    
    float pwm = (angle * 1000.0f / 180.0f) + 250.0f;
    return (int)roundf(pwm); // ËÄÉáÎåÈëµ½ÕûÊý
}


float base_x(float ts,float x_point)
{
	float angle = 0;
	angle = 135.0f - (atan(x_point/ts))*57.3f;
	return angle;
}

float base_y(float ts,float y_point)
{
	float angle = 0;
	angle = 90.0f - (atan(y_point/ts))*57.3f;
	return angle;
}

float real_arm_angle;
float real_base_angle;

float step_base;
float step_arm;

//¼¤¹â±ÊÔÚUVÖ½°åµÄÍ¶Éä¾àÀë£¬µ¥Î»ÊÇÃ×
const float ts_dis = 0.35f;

#include <stdio.h>

//¶¨Ê±Æ÷2¸üÐÂÖÐ¶Ï
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{   
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
		
	
		static float target_arm_angle = 90;		//Ä¿±êÎ»ÖÃ
		static float target_base_angle = 135;
		
		static float last_arm_angle = 90;		//ÉÏÒ»´ÎµÄÎ»ÖÃ
		static float last_base_angle = 135;
		
		static float real_arm_angle = 90;		//Êµ¼ÊÎ»ÖÃ
		static float real_base_angle= 135;
		
		static float step_base;		
		static float step_arm;
		
		static int32_t mode = 0;
		
		/////////////////////////////// Èý½ÇÐÎ»æÖÆ  Ê¹ÓÃ´ò¿ª´Ë²¿·Ö×¢ÊÍºó,ÔÙ×¢ÊÍÆäËû²¿·Ö ////////////////////////////
		static int interp_step = 0; // ²åÖµ²½Êý¼ÆÊýÆ÷
		
		const int cnt = 10;
		
		if (interp_step == 0) { // ½öÔÚ²åÖµ¿ªÊ¼Ê±¸üÐÂÄ¿±ê½Ç¶È
			if (mode <= 0) {
				target_base_angle = base_x(ts_dis, 0);
				target_arm_angle = base_y(ts_dis, 0);
			} else if (mode == 30) {
				target_base_angle = base_x(ts_dis, 0.0);
				target_arm_angle = base_y(ts_dis, 0.05);
			} else if (mode == 30*2) {
				target_base_angle = base_x(ts_dis, -0.05);
				target_arm_angle = base_y(ts_dis, 0.0);
			} else if (mode == 30*3) {
				target_base_angle = base_x(ts_dis, 0.05);
				target_arm_angle = base_y(ts_dis, 0.0);
				mode = 0; // ÖØÖÃmodeÒÔÑ­»·»æÖÆÈý½ÇÐÎ
			}

			// ¼ÆËã²åÖµ²½³¤
			step_base = (target_base_angle - last_base_angle) / cnt;
			step_arm = (target_arm_angle - last_arm_angle) / cnt;
			last_base_angle = target_base_angle;
			last_arm_angle = target_arm_angle;
		}

		// ²åÖµ
		real_arm_angle = real_arm_angle + step_arm ;
		real_base_angle = real_base_angle + step_base;

		// ½Ç¶È×ª»»ÎªPWM
		int pwmA = angle_to_pwm_180(real_arm_angle);
		int pwmB = angle_to_pwm_270(real_base_angle);

		// Êä³öµ½¶æ»ú
		TIM4->CCR3 = pwmB;
		TIM4->CCR4 = pwmA;

		// ¸üÐÂ²åÖµ²½ÊýºÍmode
		interp_step++;
		if (interp_step >= cnt) { // ²åÖµÍê³É£¬ÇÐ»»µ½ÏÂÒ»¸öÄ¿±êµã
			interp_step = 0;
			mode++;
		}
		/////////////////////////////// Èý½ÇÐÎ END ////////////////////////////
		
		/////////////////////////////// Ô²ÐÎ»æÖÆ  Ê¹ÓÃ´ò¿ª´Ë²¿·Ö×¢ÊÍºó,ÔÙ×¢ÊÍÆäËû²¿·Ö  //////////////////////////
//		static float theta = 0.0f; // Ô²µÄ²ÎÊý»¯½Ç¶È£¨»¡¶È£©
//		static const float radius = 0.06f; // Ô²µÄ°ë¾¶£¨¼ÙÉèÓëÄãµÄÆ«ÒÆÁ¿µ¥Î»Ò»ÖÂ£©
//		static const int steps = 500; // Ã¿È¦µÄ²½Êý
//		static const float theta_step = 2.0f * 3.1415926535f / steps; // Ã¿²½µÄ½Ç¶ÈÔöÁ¿

//		if (mode < steps) {
//			// ¼ÆËãÄ¿±ê½Ç¶È
//			target_base_angle = base_x(ts_dis, radius * cos(theta));
//			target_arm_angle = base_y(ts_dis, radius * sin(theta));

//			// ¼ÆËãÃ¿²½µÄÔöÁ¿
//			step_base = (target_base_angle - last_base_angle) / 100.0f;
//			step_arm = (target_arm_angle - last_arm_angle) / 100.0f;

//			// ¸üÐÂÉÏÒ»´ÎµÄ½Ç¶È
//			last_base_angle = target_base_angle;
//			last_arm_angle = target_arm_angle;

//			// Ôö¼Ó½Ç¶È²ÎÊý
//			theta += theta_step;
//			mode++;
//		} else {
//			// Ò»È¦Íê³ÉºóÖØÖÃ»ò½øÈëÆäËû×´Ì¬
//			mode = 0;
//			theta = 0.0f; // ÖØÖÃ½Ç¶ÈÒÔÖØ¸´»æÖÆÔ²ÐÎ
//		}

//		// ¼ÆËãÊµ¼Ê½Ç¶È£¨²åÖµ£©
//		real_arm_angle = last_arm_angle + step_arm;
//		real_base_angle = last_base_angle + step_base;

//		// ×ª»»Îª PWM ÐÅºÅ
//		int pwmA = angle_to_pwm_180(real_arm_angle);
//		int pwmB = angle_to_pwm_270(real_base_angle);

//		// Êä³öµ½¶æ»ú
//		TIM4->CCR3 = pwmB;
//		TIM4->CCR4 = pwmA;
		/////////////////////////////// Ô²ÐÎ END //////////////////////////
		
		/////////////////////////////// ÕýÏÒ²¨ »æÖÆ  Ê¹ÓÃ´ò¿ª´Ë²¿·Ö×¢ÊÍºó,ÔÙ×¢ÊÍÆäËû²¿·Ö  ////////////////////////////////
//		static float theta = 0.0f; // ×ø±êÖµ
//		static const float radius = 0.03f; // Ô²µÄ°ë¾¶£¨¼ÙÉèÓëÄãµÄÆ«ÒÆÁ¿µ¥Î»Ò»ÖÂ£©
//		static const int steps = 300; // Ã¿È¦µÄ²½Êý
//		static const float theta_step = 0.4/steps; // Ã¿²½µÄ½Ç¶ÈÔöÁ¿
//		
//		static uint8_t low_fre = 0;
//		
//		low_fre = !low_fre;
//		
//		if( 1 ) 
//		{
//			if (mode < steps) {
//				// ¼ÆËãÄ¿±ê½Ç¶È
//				target_base_angle = base_x(ts_dis, -0.2+theta);
//				target_arm_angle = base_y(ts_dis, radius * sin(30*3.1415926f*theta));

//				// ¼ÆËãÃ¿²½µÄÔöÁ¿
//				step_base = (target_base_angle - last_base_angle) / 100.0f;
//				step_arm = (target_arm_angle - last_arm_angle) / 100.0f;

//				// ¸üÐÂÉÏÒ»´ÎµÄ½Ç¶È
//				last_base_angle = target_base_angle;
//				last_arm_angle = target_arm_angle;

//				// Ôö¼Ó½Ç¶È²ÎÊý
//				theta += theta_step;
//				mode++;
//			} 
//			else if( mode>=steps && mode<=steps+100 )
//			{
//				mode++;
//				TIM4->CCR4 = 1000;
//				TIM4->CCR3 = 1100;
//				return;
//			}
//			else {
//				// Ò»È¦Íê³ÉºóÖØÖÃ»ò½øÈëÆäËû×´Ì¬
//				mode = 0;
//				theta = 0.0f; // ÖØÖÃ½Ç¶ÈÒÔÖØ¸´»æÖÆÔ²ÐÎ
//			}

//			// ¼ÆËãÊµ¼Ê½Ç¶È£¨²åÖµ£©
//			real_arm_angle = last_arm_angle + step_arm;
//			real_base_angle = last_base_angle + step_base;
//		}


//		// ×ª»»Îª PWM ÐÅºÅ
//		int pwmA = angle_to_pwm_180(real_arm_angle);
//		int pwmB = angle_to_pwm_270(real_base_angle);

//		// Êä³öµ½¶æ»ú
//		TIM4->CCR3 = pwmB;
//		TIM4->CCR4 = pwmA;
//		/////////////////////////////// ÕýÏÒ²¨ /////////////////////////////////
//		
		//²É¼¯µç³ØµçÑ¹
		Voltage_All+=Get_battery_volt();
		if(++Voltage_Count==100) Voltage=(float)Voltage_All/10000.0f,Voltage_All=0,Voltage_Count=0,LED=!LED;
	}
}
show.c为空
```

