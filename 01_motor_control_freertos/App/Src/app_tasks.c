#include "app_tasks.h"
#include "motor_ctl.h"
#include "uart_service.h"
#include "motor.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdio.h>

#define MOTOR_CONTROL_PERIOD_TICKS  1U
#define TELEMETRY_PERIOD_TICKS      10U

// 把浮点目标格式化为保留一位小数的字符串（不依赖 %f，避免引入浮点 printf）。
static void AppTasks_FormatValue(char *buffer, size_t size, float value)
{
    int32_t scaled = (int32_t)(value * 10.0f +
                               ((value >= 0.0f) ? 0.5f : -0.5f));
    int32_t magnitude = (scaled < 0) ? -scaled : scaled;

    (void)snprintf(buffer, size, "%s%ld.%01ld",
                   (scaled < 0) ? "-" : "",
                   (long)(magnitude / 10), (long)(magnitude % 10));
}

// 指令接收回报任务
static void AppTasks_ReportCommand(MotorControl_CommandResult_t result)
{
    char message[24];
    char value_text[16];

    switch (result.kind)
    {
        case MOTOR_CMD_SPEED:
            AppTasks_FormatValue(value_text, sizeof(value_text), result.value);
            (void)snprintf(message, sizeof(message), "SPEED %s\r\n", value_text);
            break;

        case MOTOR_CMD_POSITION:
            AppTasks_FormatValue(value_text, sizeof(value_text), result.value);
            (void)snprintf(message, sizeof(message), "POS %s\r\n", value_text);
            break;

        case MOTOR_CMD_STOP:
            (void)snprintf(message, sizeof(message), "STOP\r\n");
            break;

        case MOTOR_CMD_INVALID:
            (void)snprintf(message, sizeof(message), "INVALID COMMAND\r\n");
            break;

        // 空行不回应
        case MOTOR_CMD_NONE:
        default:
            return;
    }

    if (!UartService_SendString(message))
        Error_Handler();
}

// 可在调试器中观察超期次数，避免用串口打印影响任务节拍。
static volatile uint32_t motor_control_overrun_count;
static volatile uint32_t telemetry_overrun_count;

// 等待下一个周期的函数，避免 osDelay(1) 叠加执行耗时导致节拍越来越慢。
static void AppTasks_WaitForPeriod(uint32_t *last_wake_time,
                                   uint32_t period_ticks,
                                   volatile uint32_t *overrun_count)
{
    *last_wake_time += period_ticks;
    osStatus_t status = osDelayUntil(*last_wake_time);
    if (status == osOK)
        return;

    // 本工程的 CMSIS 封装在目标 tick 已到或已过时返回此错误。
    // 正常超期不应进入会关闭中断的 Error_Handler。
    if (status != osErrorParameter)
    {
        Error_Handler();
        return;
    }

    ++(*overrun_count);
    // 阻塞一个周期再重新计时，避免连续追赶旧节拍占满 CPU。
    if (osDelay(period_ticks) != osOK)
    {
        Error_Handler();
        return;
    }
    *last_wake_time = osKernelGetTickCount();
}

// 电机控制任务
void AppTasks_MotorControl(void)
{
    uint32_t last_wake_time = osKernelGetTickCount();

    for (;;)
    {
        MotorControl_Step();
        AppTasks_WaitForPeriod(&last_wake_time, MOTOR_CONTROL_PERIOD_TICKS,
                               &motor_control_overrun_count);
    }
}

// 串口命令接收任务：常驻阻塞在命令队列上，按行累积后统一解析。
//只接收不解析
void AppTasks_UartCommand(void)
{
    char line[32];
    uint8_t line_length = 0;
    uint8_t line_overflow = 0;
    uint8_t command;

    // 接收中断必须先挂起，否则第一条命令就会丢
    if (!UartService_StartReceive())
        Error_Handler();
    if (!UartService_SendString("UART4 ready\r\n"))
        Error_Handler();

    for (;;)
    {
        if (UartService_ReceiveCommand(&command, osWaitForever) != osOK)
            Error_Handler();

        // 回车换行代表一条命令结束
        if ((command != '\r') && (command != '\n'))
        {
            // 超长行直接丢弃，避免溢出；仍继续累积直到行尾好一并报错
            if (line_length < (sizeof(line) - 1U))
                line[line_length++] = (char)command;
            else
                line_overflow = 1;
            continue;
        }

        line[line_length] = '\0';
        line_length = 0;

        if (line_overflow)
        {
            line_overflow = 0;
            if (!UartService_SendString("INVALID COMMAND\r\n"))
                Error_Handler();
            continue;
        }

        // 登记目标状态，实际下发由 1ms 控制任务完成
        AppTasks_ReportCommand(MotorControl_HandleCommandLine(line));
    }
}

// 遥测回传任务
void AppTasks_Telemetry(void)
{
    uint32_t last_wake_time = osKernelGetTickCount();
    uint32_t last_receive_error_count = UartService_GetReceiveErrorCount();

    for (;;)
    {
        uint32_t receive_error_count = UartService_GetReceiveErrorCount();
        if (receive_error_count != last_receive_error_count)
        {
            last_receive_error_count = receive_error_count;
            if (!UartService_SendString("UART RX ERROR\r\n"))
                Error_Handler();
        }

        if (!UartService_SendTelemetry(Motor_1.SpeedMeasure,
                                       Motor_1.PositionMeasure))
            Error_Handler();

        AppTasks_WaitForPeriod(&last_wake_time, TELEMETRY_PERIOD_TICKS,
                               &telemetry_overrun_count);
    }
}
