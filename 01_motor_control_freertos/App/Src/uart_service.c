#include "uart_service.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart4;

static osMessageQueueId_t command_queue;
static osMutexId_t uart_tx_mutex;
static uint8_t uart4_rx_byte;
static volatile uint32_t receive_error_count;

// 绑定 CubeMX 创建的 cmdQue，并创建串口发送互斥量。
// 返回 0 表示入参无效或互斥量创建失败。
uint8_t UartService_Init(osMessageQueueId_t queue)
{
    if (queue == NULL)
        return 0;

    command_queue = queue;
    uart_tx_mutex = osMutexNew(NULL);
    return (uart_tx_mutex != NULL);
}

// 挂起单字节中断接收；每收到 1 字节就会进一次 HAL_UART_RxCpltCallback
uint8_t UartService_StartReceive(void)
{
    return (HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1) == HAL_OK);
}

// 阻塞等待一条命令字节，由 UartCmdTask 调用（timeout 传 osWaitForever 即常驻等待）
osStatus_t UartService_ReceiveCommand(uint8_t *command, uint32_t timeout)
{
    if ((command_queue == NULL) || (command == NULL))
        return osErrorParameter;

    return osMessageQueueGet(command_queue, command, NULL, timeout);
}

// 所有串口输出统一走这里。加锁是为了让命令回显与遥测这两个任务
// 不会把各自的报文交叉着发出去，保证上位机按行解析时不会串包。
uint8_t UartService_SendString(const char *text)
{
    HAL_StatusTypeDef uart_status;
    osStatus_t mutex_status;

    if ((text == NULL) || (uart_tx_mutex == NULL))
        return 0;

    mutex_status = osMutexAcquire(uart_tx_mutex, osWaitForever);
    if (mutex_status != osOK)
        return 0;

    uart_status = HAL_UART_Transmit(&huart4,
                                    (uint8_t *)text,
                                    (uint16_t)strlen(text),
                                    100);
    mutex_status = osMutexRelease(uart_tx_mutex);

    return ((uart_status == HAL_OK) && (mutex_status == osOK));
}

// 按 "速度,位置\r\n" 格式发送遥测。
// 这里手工做定点拼接（乘 10 后取整再插小数点）而不是 printf("%.1f")，
// 是为了不依赖 newlib 的浮点格式化，省下可观的 Flash 与栈开销。
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

uint32_t UartService_GetReceiveErrorCount(void)
{
    return receive_error_count;
}

// UART 接收完成中断。中断里只做两件事：把字节丢进队列、重新挂起接收。
// 用超时 0 的 osMessageQueuePut，队列满时直接丢弃并计数，绝不在这里阻塞。
// 命令的解析与回显都留给 UartCmdTask，保持中断尽可能短。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
        return;

    if ((command_queue == NULL) ||
        (osMessageQueuePut(command_queue, &uart4_rx_byte, 0, 0) != osOK))
        receive_error_count++;

    if (HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1) != HAL_OK)
        receive_error_count++;
}

// 接收出错（典型是 ORE 溢出）后 HAL 会停掉接收，必须在这里重新挂起，
// 否则串口从此收不到任何数据，而且不会有任何报错提示。
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
        return;

    if (HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1) != HAL_OK)
        receive_error_count++;
}
