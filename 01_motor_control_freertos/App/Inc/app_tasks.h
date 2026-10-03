//
// Created by jedol on 2026/10/3.
//

#ifndef TEST9_9_APP_TASKA_H
#define TEST9_9_APP_TASKA_H

// 1ms 周期：套用目标状态并跑串级 PID，经 CAN 下发 PWM
void AppTasks_MotorControl(void);

// 事件驱动：阻塞等待串口命令队列，解析并回显
void AppTasks_UartCommand(void);

// 10ms 周期：上报速度/位置遥测与接收错误计数
void AppTasks_Telemetry(void);

#endif //TEST9_9_APP_TASKA_H
