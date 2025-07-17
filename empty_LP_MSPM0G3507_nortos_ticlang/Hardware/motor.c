#include "motor.h"
#include "stdio.h"
#include <math.h>
#include <stdbool.h>
#include "stdlib.h"
// PID参数（可根据调试调整）
float Velcity_Kp = 1.00, Velcity_Ki = 0.5, Velcity_Kd = 0.0;

// 外部变量声明
extern int32_t Get_Encoder_countA, encoderA_cnt, Get_Encoder_countB, encoderB_cnt;

// 距离控制相关全局变量
static int start_encoder_A = 0;
static int start_encoder_B = 0;
static volatile bool distance_control_active = false;

// 电机校准系数（可根据实际调试后微调）
static float motor_A_calibration = 1.0f;
static float motor_B_calibration = 1.0f;

// ==================== PID速度环 ====================

int Velocity_A(int TargetVelocity, int CurrentVelocity)
{
    int Bias;
    static int ControlVelocityA = 0, Last_biasA = 0;

    Bias = TargetVelocity - CurrentVelocity;
    ControlVelocityA += Velcity_Ki * (Bias - Last_biasA) + Velcity_Kp * Bias;
    Last_biasA = Bias;
    if (ControlVelocityA > 3600) ControlVelocityA = 3600;
    else if (ControlVelocityA < -3600) ControlVelocityA = -3600;
    return ControlVelocityA;
}

int Velocity_B(int TargetVelocity, int CurrentVelocity)
{
    int Bias;
    static int ControlVelocityB = 0, Last_biasB = 0;

    Bias = TargetVelocity - CurrentVelocity;
    ControlVelocityB += Velcity_Ki * (Bias - Last_biasB) + Velcity_Kp * Bias;
    Last_biasB = Bias;
    if (ControlVelocityB > 3600) ControlVelocityB = 3600;
    else if (ControlVelocityB < -3600) ControlVelocityB = -3600;
    return ControlVelocityB;
}

// ==================== PWM输出 ====================

void Set_PWM(int pwma, int pwmb)
{
    // 应用校准补偿
    int calibrated_pwma = (int)(pwma * motor_A_calibration);
    int calibrated_pwmb = (int)(pwmb * motor_B_calibration);

    // A电机
    if (calibrated_pwma > 0) {
        DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_12_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_13_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(calibrated_pwma), GPIO_PWM_0_C0_IDX);
    } else if (calibrated_pwma < 0) {
        DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_13_PIN);
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_12_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(calibrated_pwma), GPIO_PWM_0_C0_IDX);
    } else {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_12_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_13_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    }

    // B电机
    if (calibrated_pwmb > 0) {
        DL_GPIO_setPins(BIN1_PORT, BIN1_Pin_Bin1_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_Pin_Bin2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(calibrated_pwmb), GPIO_PWM_0_C1_IDX);
    } else if (calibrated_pwmb < 0) {
        DL_GPIO_setPins(BIN2_PORT, BIN2_Pin_Bin2_PIN);
        DL_GPIO_clearPins(BIN1_PORT, BIN1_Pin_Bin1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, ABS(calibrated_pwmb), GPIO_PWM_0_C1_IDX);
    } else {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_Pin_Bin1_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_Pin_Bin2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C1_IDX);
    }
}

// ==================== 距离与运动控制 ====================

void Reset_Distance_Counter(void)
{
    start_encoder_A = encoderA_cnt;
    start_encoder_B = encoderB_cnt;
}

float Get_Current_Distance(void)
{
    int avg_encoder_diff = ((encoderA_cnt - start_encoder_A) + (encoderB_cnt - start_encoder_B)) / 2;
    float revolutions = (float)avg_encoder_diff / ENCODER_PULSES_PER_REV;
    return revolutions * WHEEL_CIRCUMFERENCE;
}

void Stop_Motors(void)
{
    Set_PWM(0, 0);
    distance_control_active = false;
}

// 支持正反向的距离运动
void Move_Forward_Distance_Straight(float distance_cm)
{
    Reset_Distance_Counter();
    distance_control_active = true;

    float target_revolutions = fabsf(distance_cm) / WHEEL_CIRCUMFERENCE;
    int target_pulses = (int)(target_revolutions * ENCODER_PULSES_PER_REV);
    int direction = (distance_cm >= 0) ? 1 : -1;
    int current_distance_pulses = 0;
    int base_speed = TARGET_SPEED * direction;

    // 用于直线校正的比例参数
    float straight_kp = 0.5f;
    int direction_error = 0;

    while (distance_control_active && abs(current_distance_pulses) < target_pulses)
    {
        int moved_A = (encoderA_cnt - start_encoder_A) * direction;
        int moved_B = (encoderB_cnt - start_encoder_B) * direction;
        current_distance_pulses = (moved_A + moved_B) / 2;

        // 方向误差调整
        direction_error = moved_A - moved_B;
        int speed_correction = (int)(straight_kp * direction_error);

        int speed_A = base_speed - speed_correction;
        int speed_B = base_speed + speed_correction;

        // 限制速度范围
        if (abs(speed_A) < 50) speed_A = 50 * direction;
        if (abs(speed_A) > 500) speed_A = 500 * direction;
        if (abs(speed_B) < 50) speed_B = 50 * direction;
        if (abs(speed_B) > 500) speed_B = 500 * direction;

        // 距离接近终点时减速
        int remaining_pulses = target_pulses - abs(current_distance_pulses);
        if (remaining_pulses < (target_pulses / 4)) {
            float decel_factor = (float)remaining_pulses / (target_pulses / 4);
            speed_A = (int)(speed_A * decel_factor);
            speed_B = (int)(speed_B * decel_factor);
            if (abs(speed_A) < 50) speed_A = 50 * direction;
            if (abs(speed_B) < 50) speed_B = 50 * direction;
        }

        int current_vel_A = Get_Encoder_countA * direction;
        int current_vel_B = Get_Encoder_countB * direction;
        int pwm_A = Velocity_A(speed_A, current_vel_A);
        int pwm_B = Velocity_B(speed_B, current_vel_B);

        Set_PWM(pwm_A, pwm_B);

        delay_ms(10);
    }
    Stop_Motors();
}

void Move_Forward_Distance(float distance_cm)
{
    Move_Forward_Distance_Straight(distance_cm);
}

void Move_Forward_20cm(void)
{
    Move_Forward_Distance_Straight(20.0f);
}

// ==================== 电机校准（可选） ====================

void calibrate_motors(void)
{
    uart0_send_string("Starting motor calibration...\r\n");

    Reset_Distance_Counter();
    int test_pwm = 300;
    int test_time = 200; // 2秒

    // 同时运行两个电机
    for (int i = 0; i < test_time; i++) {
        Set_PWM(test_pwm, test_pwm);
        delay_ms(10);
    }
    Set_PWM(0, 0);

    int final_encoder_A = encoderA_cnt - start_encoder_A;
    int final_encoder_B = encoderB_cnt - start_encoder_B;

    char buffer[100];
    sprintf(buffer, "Calibration - Motor A: %d, Motor B: %d\r\n", final_encoder_A, final_encoder_B);
    uart0_send_string(buffer);

    // 以较慢的电机为基准
    if (abs(final_encoder_A) > abs(final_encoder_B)) {
        motor_A_calibration = (float)abs(final_encoder_B) / abs(final_encoder_A);
        motor_B_calibration = 1.0f;
    } else {
        motor_A_calibration = 1.0f;
        motor_B_calibration = (float)abs(final_encoder_A) / abs(final_encoder_B);
    }

    sprintf(buffer, "Calibration factors - A: %.3f, B: %.3f\r\n", motor_A_calibration, motor_B_calibration);
    uart0_send_string(buffer);
}