/*
 * Example: How to use the new motor calibration functions
 * 示例：如何使用新的电机校准功能
 * 
 * This example shows how to integrate the new debugging and calibration
 * functions into your motor control application.
 * 此示例展示如何将新的调试和校准功能集成到电机控制应用中。
 */

#include "motor.h"

void example_motor_calibration_usage(void)
{
    // Example 1: Print current motor status for debugging
    // 示例1：打印当前电机状态用于调试
    printf("=== Initial Motor Status ===\n\r");
    Motor_Debug_Print();
    
    // Example 2: Get debug info programmatically 
    // 示例2：编程方式获取调试信息
    int encA, encB, pwmA, pwmB;
    Motor_Get_Debug_Info(&encA, &encB, &pwmA, &pwmB);
    
    // Check if motors are unbalanced
    // 检查电机是否不平衡
    if(abs(encA - encB) > 3) {
        printf("Motor imbalance detected! EncoderA: %d, EncoderB: %d\n\r", encA, encB);
        
        // Example 3: Auto-calibration based on encoder difference
        // 示例3：基于编码器差异的自动校准
        if(encA > encB) {
            // Motor A is faster, reduce its gain
            // 电机A更快，减少其增益
            printf("Reducing Motor A gain...\n\r");
            Motor_Calibrate_A(0.9, 0.45, 0.0);
        } else {
            // Motor B is faster, reduce its gain  
            // 电机B更快，减少其增益
            printf("Reducing Motor B gain...\n\r");
            Motor_Calibrate_B(0.9, 0.45, 0.0);
        }
    }
    
    // Example 4: Manual fine-tuning
    // 示例4：手动精细调节
    if(encA - encB > 5) {
        printf("Large difference detected, applying aggressive tuning...\n\r");
        Motor_Calibrate_A(0.8, 0.4, 0.0);  // More aggressive reduction
    }
    
    // Example 5: Restore to default if needed
    // 示例5：如需要可恢复默认值
    // Motor_Calibrate_A(1.0, 0.5, 0.0);  // Default values
    // Motor_Calibrate_B(1.0, 0.5, 0.0);  // Default values
    
    printf("=== After Calibration ===\n\r");
    Motor_Debug_Print();
}

/*
 * Example: Integration into timer interrupt
 * 示例：集成到定时器中断中
 */
void example_timer_integration(void)
{
    static int calibration_counter = 0;
    
    // Run calibration check every 100 timer cycles
    // 每100个定时器周期运行一次校准检查
    calibration_counter++;
    if(calibration_counter >= 100) {
        calibration_counter = 0;
        
        // Auto-adjust if motors are consistently unbalanced
        // 如果电机持续不平衡则自动调整
        int encA, encB, pwmA, pwmB;
        Motor_Get_Debug_Info(&encA, &encB, &pwmA, &pwmB);
        
        static int unbalance_count = 0;
        if(abs(encA - encB) > 2) {
            unbalance_count++;
            if(unbalance_count > 5) {  // Consistent imbalance
                printf("Auto-calibrating due to persistent imbalance...\n\r");
                if(encA > encB) {
                    Motor_Calibrate_A(0.95, 0.48, 0.0);
                } else {
                    Motor_Calibrate_B(0.95, 0.48, 0.0);
                }
                unbalance_count = 0;
            }
        } else {
            unbalance_count = 0;  // Reset counter if balanced
        }
    }
}

/*
 * Example: UART command interface for manual calibration
 * 示例：用于手动校准的UART命令接口
 */
void example_uart_calibration_commands(char* command)
{
    // Simple command parser for calibration
    // 简单的校准命令解析器
    if(strncmp(command, "CAL_A", 5) == 0) {
        float kp, ki, kd;
        if(sscanf(command, "CAL_A %f %f %f", &kp, &ki, &kd) == 3) {
            Motor_Calibrate_A(kp, ki, kd);
            printf("Motor A calibrated successfully\n\r");
        }
    }
    else if(strncmp(command, "CAL_B", 5) == 0) {
        float kp, ki, kd;
        if(sscanf(command, "CAL_B %f %f %f", &kp, &ki, &kd) == 3) {
            Motor_Calibrate_B(kp, ki, kd);
            printf("Motor B calibrated successfully\n\r");
        }
    }
    else if(strcmp(command, "DEBUG") == 0) {
        Motor_Debug_Print();
    }
    else if(strcmp(command, "RESET") == 0) {
        Motor_Calibrate_A(1.0, 0.5, 0.0);
        Motor_Calibrate_B(1.0, 0.5, 0.0);
        printf("Motors reset to default parameters\n\r");
    }
}

/*
 * Usage commands via UART:
 * 通过UART使用的命令：
 * 
 * CAL_A 1.2 0.6 0.0    - Calibrate motor A with Kp=1.2, Ki=0.6, Kd=0.0
 * CAL_B 0.9 0.4 0.0    - Calibrate motor B with Kp=0.9, Ki=0.4, Kd=0.0
 * DEBUG                - Print current motor status
 * RESET                - Reset both motors to default parameters
 */