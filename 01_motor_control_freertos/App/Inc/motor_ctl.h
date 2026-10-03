//
// Created by jedol on 2026/10/3.
//

#ifndef TEST9_9_MOTOR_CTR_H
#define TEST9_9_MOTOR_CTR_H

#include <stdint.h>

// 初始化 PID 参数并创建状态互斥量，返回 0 表示失败（致命）
uint8_t MotorControl_Init(void);

// 解析一条命令字符并登记期望状态（不直接改电机，由 Step 套用）
// f/F 正转  r/R 反转  p/P 位置正转一步  n/N 位置反转一步  s/S 停机
void MotorControl_HandleCommand(uint8_t command);

// 1ms 控制节拍：套用目标 -> 串级 PID -> CAN 下发 PWM
void MotorControl_Step(void);

#endif //TEST9_9_MOTOR_CTR_H
