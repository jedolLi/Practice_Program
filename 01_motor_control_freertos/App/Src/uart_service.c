#include "uart_service.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart4;

static osMessageQueueId_t command_queue;
static osMutexId_t uart_tx_mutex;
static uint8_t uart4_rx_byte;
static volatile uint32_t receive_error_count;

// 初始化串口服务，创建保护串口输出的互斥量。
uint8_t UartService_Init(osMessageQueueId_t queue)
{
    if (queue == NULL)
        return 0;

    command_queue = queue;
    uart_tx_mutex = osMutexNew(NULL);
    return (uart_tx_mutex != NULL);
}

// 单字节中断接收函数
uint8_t UartService_StartReceive(void)
{
    return (HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1) == HAL_OK);
}

// 阻塞等待一条命令字节函数
osStatus_t UartService_ReceiveCommand(uint8_t *command, uint32_t timeout)
{
    if ((command_queue == NULL) || (command == NULL))
        return osErrorParameter;

    return osMessageQueueGet(command_queue, command, NULL, timeout);  // 阻塞等待一条命令字节
}

//统一的串口输出函数
uint8_t UartService_SendString(const char *text)
{
    HAL_StatusTypeDef uart_status; 
    osStatus_t mutex_status;  //为了防止多任务同时调用串口输出函数，导致输出混乱，使用互斥量保护串口输出

    if ((text == NULL) || (uart_tx_mutex == NULL))
        return 0;

    //抢锁逻辑，如果uart_tx_mutex被锁上，进入永久等待，任务进入阻塞态
    mutex_status = osMutexAcquire(uart_tx_mutex, osWaitForever);  
    if (mutex_status != osOK)
        return 0;

    uart_status = HAL_UART_Transmit(&huart4,
                                    (uint8_t *)text,
                                    (uint16_t)strlen(text),
                                    100);
    mutex_status = osMutexRelease(uart_tx_mutex); //解锁

    return ((uart_status == HAL_OK) && (mutex_status == osOK)); 
}

// 发送回传数据的函数
uint8_t UartService_SendTelemetry(float speed, float position)
{
    char buffer[32];
    int32_t speed_tenths = (int32_t)(speed * 10.0f +
                                      ((speed >= 0.0f) ? 0.5f : -0.5f));
    int32_t speed_magnitude = (speed_tenths < 0) ?
                              -speed_tenths : speed_tenths;
    int length = snprintf(buffer, sizeof(buffer), "%s%ld.%01ld,%ld\r\n",
                          (speed_tenths < 0) ? "-" : "",
                          (long)(speed_magnitude / 10),
                          (long)(speed_magnitude % 10),
                          (long)(int32_t)position);

    if ((length < 0) || ((size_t)length >= sizeof(buffer)))
        return 0;

    return UartService_SendString(buffer);
}

// 获取 UART 接收错误计数
uint32_t UartService_GetReceiveErrorCount(void)
{
    return receive_error_count;
}

// UART 接收完成中断：把字节丢进队列、重新挂起接收。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
        return;

    if ((command_queue == NULL) ||
        (osMessageQueuePut(command_queue, &uart4_rx_byte, 0, 0) != osOK)) 
        //此处直接把osMessageQueuePut函数放到if语句的判断标准里面执行，执行完成之后顺便判断一下是不是osOK
        receive_error_count++;

    if (HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1) != HAL_OK)
    //同上，直接在条件语句执行receiveIT，顺便判断是否为HAL_OK
        receive_error_count++;
}

// 接收出错（典型是 ORE 溢出）后 HAL 会停掉接收，必须在这里重新挂起
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
        return;

    if (HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1) != HAL_OK)
        receive_error_count++;
}
