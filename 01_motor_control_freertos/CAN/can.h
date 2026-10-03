#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan2;

// 配置 CAN2 滤波器（当前掩码全 0，即全部 ID 放行）
void CAN_Filter_init(void);

// 启动 CAN2 并打开 RX FIFO0 接收中断，必须在 osKernelStart 之前调用
void CAN_Start(void);

// 发送一帧标准数据帧，id 为 11 位标准 ID
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len);

// 解析 8 字节 RM3508 反馈报文，结果写入 Motor_1
void get_motor_value(uint8_t *rxData);

#ifdef __cplusplus
}
#endif

#endif
