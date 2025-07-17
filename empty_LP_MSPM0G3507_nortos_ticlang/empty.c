#include "ti_msp_dl_config.h"
#include "Hardware/led.h"
#include "Hardware/board.h"
#include "Hardware/motor.h"
#include "Hardware/encoder.h"

int main(void)
{
    SYSCFG_DL_init();
    board_init();
    
    delay_ms(2000);
    uart0_send_string("System initialized\r\n");
    
    // 首先进行电机校准
    calibrate_motors();
    delay_ms(2000);
    
    // 测试直线行驶
    uart0_send_string("Starting 20cm straight line test\r\n");
    Move_Forward_20cm();
    delay_ms(5000);
    
    while(1)
    {
        Move_Forward_Distance_Straight(20.0f);
        delay_ms(5000);
    }
}