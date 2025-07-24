#include "board.h"
#include "stdio.h"
#include "string.h"
#define RE_0_BUFF_LEN_MAX 128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};  // 接收缓冲区
volatile uint16_t recv0_length = 0;                     // 接收数据长度
volatile uint8_t  recv0_flag = 0;                       // 接收完成标志

// 微秒级延时
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // 计算所需的SysTick计数值
    ticks = __us * (32000000 / 1000000);
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            if (tcnt >= ticks)
                break;
        }
    }
}

// 毫秒级延时
void delay_ms(unsigned long ms) 
{
    delay_us(ms * 1000);
}

void delay_1us(unsigned long __us) { delay_us(__us); }
void delay_1ms(unsigned long ms) { delay_ms(ms); }

// 发送单个字符
void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

// 发送字符串
void uart0_send_string(char* str)
{
    while (*str != 0 && str != 0)
    {
        uart0_send_char(*str++);
    }
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

// 禁用半主机模式
void _sys_exit(int x)
{
    x = x;
}
#endif

// printf 重定向到串口
int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
    return ch;
}

// fputs 实现
int fputs(const char* restrict s, FILE* restrict stream)
{
    uint16_t i, len;
    len = strlen(s);
    for (i = 0; i < len; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, s[i]);
    }
    return len;
}

// puts 实现（带换行）
int puts(const char *_ptr)
{
    int count = fputs(_ptr, stdout);
    count += fputs("\n", stdout);
    return count;
}

// 串口中断服务函数
void UART_0_INST_IRQHandler(void)
{
    uint8_t receivedData = 0;

    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:  // 接收中断
            receivedData = DL_UART_Main_receiveData(UART_0_INST);

            if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
            {
                recv0_buff[recv0_length++] = receivedData;

                // 回显接收到的字符
                uart0_send_char(receivedData);
            }
            else
            {
                recv0_length = 0;
            }

            recv0_flag = 1; // 设置接收完成标志
            break;

        default: // 其他中断不处理
            break;
    }
}
