# 电机校准和调试指南 / Motor Calibration and Debugging Guide

## 问题描述 / Problem Description
小车运行时一直转弯而不是直走，这是由于两个电机的PID参数相同但电机特性不同造成的。
The cart turns instead of going straight due to identical PID parameters for motors with different characteristics.

## 新增功能 / New Features

### 1. 独立PID参数 / Separate PID Parameters
每个电机现在有独立的PID参数：
- `Motor_A_PID` - 左电机 / Left motor
- `Motor_B_PID` - 右电机 / Right motor

### 2. 调试功能 / Debugging Functions

#### `Motor_Debug_Print()`
实时输出电机状态信息 / Real-time motor status output:
```
Motor Debug Info:
EncoderA: 8, EncoderB: 7
PWMA: 150, PWMB: 180
Motor_A PID: Kp=1.00, Ki=0.50, Kd=0.00
Motor_B PID: Kp=1.20, Ki=0.55, Kd=0.00
====================
```

#### `Motor_Get_Debug_Info(int *encoderA, int *encoderB, int *pwmA, int *pwmB)`
编程方式获取调试信息 / Programmatic access to debug data:
```c
int encA, encB, pwmA, pwmB;
Motor_Get_Debug_Info(&encA, &encB, &pwmA, &pwmB);
```

### 3. 运行时校准 / Runtime Calibration

#### `Motor_Calibrate_A(float kp, float ki, float kd)`
调整左电机PID参数 / Adjust left motor PID:
```c
Motor_Calibrate_A(1.2, 0.6, 0.0);  // 增加Kp和Ki值
```

#### `Motor_Calibrate_B(float kp, float ki, float kd)`
调整右电机PID参数 / Adjust right motor PID:
```c
Motor_Calibrate_B(0.9, 0.4, 0.0);  // 减少Kp和Ki值
```

## 使用步骤 / Usage Steps

### 步骤1：观察初始行为 / Step 1: Observe Initial Behavior
1. 运行程序并观察小车行为 / Run program and observe cart behavior
2. 查看串口输出的调试信息 / Check debug output on serial port
3. 记录编码器值差异 / Note encoder value differences

### 步骤2：分析问题 / Step 2: Analyze Problem
- 如果小车向左转：右电机可能太快 / If cart turns left: right motor may be too fast
- 如果小车向右转：左电机可能太快 / If cart turns right: left motor may be too fast
- 观察PWM值差异 / Observe PWM value differences

### 步骤3：调整参数 / Step 3: Adjust Parameters

#### 电机太快时 / When motor is too fast:
```c
// 减少Kp和Ki值 / Reduce Kp and Ki values
Motor_Calibrate_A(0.8, 0.4, 0.0);
```

#### 电机太慢时 / When motor is too slow:
```c
// 增加Kp和Ki值 / Increase Kp and Ki values  
Motor_Calibrate_B(1.3, 0.7, 0.0);
```

### 步骤4：测试和优化 / Step 4: Test and Optimize
1. 进行小幅调整 / Make small adjustments (±0.1 for Kp, ±0.05 for Ki)
2. 测试直线行驶效果 / Test straight-line performance
3. 重复直到满意 / Repeat until satisfactory

## 调试提示 / Debugging Tips

### 常见问题和解决方案 / Common Issues and Solutions

#### 问题1：编码器值相差太大 / Issue 1: Large encoder difference
```
WARNING: Large encoder difference detected!
Consider calibrating motors for better straight-line performance
```
**解决方案 / Solution**: 调整较快电机的PID参数 / Adjust PID of faster motor

#### 问题2：PWM值饱和 / Issue 2: PWM saturation
如果PWM值达到±3600的限制 / If PWM reaches ±3600 limit:
- 减少Ki值防止积分饱和 / Reduce Ki to prevent integral windup
- 检查目标速度是否合理 / Check if target speed is reasonable

#### 问题3：振荡行为 / Issue 3: Oscillating behavior
- 减少Kp值 / Reduce Kp value
- 考虑添加Kd项（阻尼） / Consider adding Kd term (damping)

## 参数调节指导 / Parameter Tuning Guidelines

### Kp (比例项 / Proportional)
- 影响响应速度 / Affects response speed
- 过大：振荡 / Too high: oscillation
- 过小：响应慢 / Too low: slow response
- 建议范围：0.5-2.0 / Recommended range: 0.5-2.0

### Ki (积分项 / Integral)
- 消除稳态误差 / Eliminates steady-state error
- 过大：积分饱和 / Too high: integral windup
- 过小：稳态误差 / Too low: steady-state error  
- 建议范围：0.1-1.0 / Recommended range: 0.1-1.0

### Kd (微分项 / Derivative)
- 提供阻尼 / Provides damping
- 当前未使用，可用于减少振荡 / Currently unused, can reduce oscillation
- 建议范围：0.0-0.1 / Recommended range: 0.0-0.1

## 兼容性说明 / Compatibility Notes
- 原有代码继续正常工作 / Existing code continues to work
- `Velocity_A()` 和 `Velocity_B()` 函数保持相同接口 / Functions maintain same interface
- 新功能为可选使用 / New features are optional to use