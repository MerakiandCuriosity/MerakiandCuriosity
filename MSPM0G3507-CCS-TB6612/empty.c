/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "led.h"
#include "board.h"
#include "motor.h"
int Get_Encoder_countA,encoderA_cnt,Get_Encoder_countB,encoderB_cnt,PWMA,PWMB;
static int debug_counter = 0;  // Counter for periodic detailed debug output

int main(void)
{
    SYSCFG_DL_init();
    DL_Timer_startCounter(PWM_0_INST);//启动PWM定时器
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);//清除中断标志位
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);//串口中断
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    
    printf("Motor Debug System Initialized\n\r");
    printf("Enhanced debugging enabled with separate PID control\n\r");
    
    while (1) {
        // Basic encoder output (original functionality)
        printf("%d %d\n\r",encoderA_cnt,encoderB_cnt);
        
        // Enhanced debugging every 50 iterations (~1 second at typical loop speed)
        debug_counter++;
        if(debug_counter >= 50) {
            Motor_Debug_Print();
            debug_counter = 0;
            
            // Example: Automatically tune Motor B if there's significant difference
            if(abs(encoderA_cnt - encoderB_cnt) > 5) {
                printf("WARNING: Large encoder difference detected!\n\r");
                printf("Consider calibrating motors for better straight-line performance\n\r");
            }
        }
    }
}


void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
            encoderA_cnt = -Get_Encoder_countA;
            encoderB_cnt = Get_Encoder_countB;
            Get_Encoder_countA = 0;
            Get_Encoder_countB = 0;
            LED_Flash(100);//LED1��˸
            PWMA = Velocity_A(8,encoderA_cnt);
            PWMB = Velocity_B(8,encoderB_cnt);
            Set_PWM(PWMA,PWMB);
        }

    }
}




