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
    
    // 输出完成信息
    uart0_send_string("20cm straight line movement completed!\r\n");
    uart0_send_string("System entering waiting state...\r\n");
    
    // 停止所有电机运动
    Stop_Motors();
    
    // 进入无限循环等待状态，保持系统运行但不执行动作
    while(1)
    {
        delay_ms(1000);  // 1秒延时，保持系统活跃状态
    }
}