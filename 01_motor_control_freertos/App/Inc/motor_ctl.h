//
// Created by jedol on 2026/10/3.
//

#ifndef TEST9_9_MOTOR_CTR_H
#define TEST9_9_MOTOR_CTR_H

#include <stdint.h>

// 一条命令行的解析结果类别，调用方据此决定回显内容
typedef enum
{
    MOTOR_CMD_NONE = 0,  // 空行，无需处理与回显
    MOTOR_CMD_SPEED,     // 速度环目标，value 为生效转速（rpm，可负）
    MOTOR_CMD_POSITION,  // 位置环目标，value 为相对步进角度（度，可负）
    MOTOR_CMD_STOP,      // 停机
    MOTOR_CMD_INVALID    // 非法命令
} MotorCommandKind_t;

// 记录解析结果的类别与生效数值，供回显使用
typedef struct
{
    MotorCommandKind_t kind;
    float value;
} MotorControl_CommandResult_t;

// 初始化 PID 参数并创建状态互斥量，返回 0 表示失败（致命）
uint8_t MotorControl_Init(void);

// 解析一整行命令并登记期望状态（不直接改电机，由 Step 套用），
// 返回解析结果供调用方回显。
//   f/F[数值]  速度环（方向为 +），无数值用默认转速，如 f100 / f-100
//   r/R[数值]  速度环（方向为 -），无数值用默认转速，如 r100
//   p/P[数值]  位置环相对正转，无数值用默认步长，如 p270
//   n/N[数值]  位置环相对反转，无数值用默认步长，如 n270
//   s/S        停机
// 数值允许可选正负号与小数点，生效值 = 方向 × 数值；行尾只允许空白，否则非法。
MotorControl_CommandResult_t MotorControl_HandleCommandLine(const char *line);

// 1ms 控制节拍：套用目标 -> 串级 PID -> CAN 下发 PWM
void MotorControl_Step(void);

#endif //TEST9_9_MOTOR_CTR_H
