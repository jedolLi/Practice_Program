//
// Created by jedol on 2026/10/3.
//

#ifndef TEST9_9_UART_SERVICE_H
#define TEST9_9_UART_SERVICE_H

#include <stdint.h>
#include "cmsis_os2.h"

// 绑定 CubeMX 的 cmdQue 并创建发送互斥量，返回 0 表示失败（致命）
uint8_t UartService_Init(osMessageQueueId_t command_queue);

// 挂起 UART4 单字节中断接收，返回 0 表示失败（致命）
uint8_t UartService_StartReceive(void);

// 阻塞取一条命令字节；timeout 传 osWaitForever 表示常驻等待
osStatus_t UartService_ReceiveCommand(uint8_t *command, uint32_t timeout);

// 加锁发送一段字符串（命令回显等），返回 0 表示发送失败
uint8_t UartService_SendString(const char *text);

// 加锁发送 "速度,位置\r\n" 遥测报文，返回 0 表示发送失败
uint8_t UartService_SendTelemetry(float speed, float position);

// 累计的接收错误次数（队列满丢包、接收重启失败等），供遥测上报
uint32_t UartService_GetReceiveErrorCount(void);

#endif //TEST9_9_UART_SERVICE_H
