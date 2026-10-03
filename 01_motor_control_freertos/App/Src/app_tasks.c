#include "app_tasks.h"
#include "motor_ctl.h"
#include "uart_service.h"
#include "motor.h"
#include "cmsis_os2.h"
#include "main.h"

#define MOTOR_CONTROL_PERIOD_TICKS  1U
#define TELEMETRY_PERIOD_TICKS      10U

// 电机控制任务：1ms 一个节拍跑串级 PID 并下发 CAN。
// 用 osDelayUntil 累加"绝对唤醒时刻"而不是 osDelay(1)：
// 后者会把每次循环的执行耗时也叠加进去，长时间跑下来节拍会越来越慢。
void AppTasks_MotorControl(void)
{
    uint32_t last_wake_time = osKernelGetTickCount();

    for (;;)
    {
        MotorControl_Step();
        last_wake_time += MOTOR_CONTROL_PERIOD_TICKS;
        if (osDelayUntil(last_wake_time) != osOK)
            Error_Handler();
    }
}

// 串口命令任务：常驻阻塞在命令队列上，收到一条处理一条。
// 注意这里没有 osDelay —— 任务靠队列阻塞让出 CPU，命令到达时立刻被唤醒，
// 比轮询方式响应更快且不占用任何 CPU 时间。
void AppTasks_UartCommand(void)
{
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

        if ((command == '\r') || (command == '\n'))
            continue;

        // 登记目标状态，实际下发由 1ms 控制任务完成
        MotorControl_HandleCommand(command);
        switch (command)
        {
            case 'f':
            case 'F':
                if (!UartService_SendString("SPEED FWD\r\n"))
                    Error_Handler();
                break;
            case 'r':
            case 'R':
                if (!UartService_SendString("SPEED REV\r\n"))
                    Error_Handler();
                break;
            case 'p':
            case 'P':
                if (!UartService_SendString("POS FWD\r\n"))
                    Error_Handler();
                break;
            case 'n':
            case 'N':
                if (!UartService_SendString("POS REV\r\n"))
                    Error_Handler();
                break;
            case 's':
            case 'S':
                if (!UartService_SendString("STOP\r\n"))
                    Error_Handler();
                break;
            default:
                if (!UartService_SendString("INVALID COMMAND\r\n"))
                    Error_Handler();
                break;
        }
    }
}

// 遥测任务：10ms 发送一次"速度,位置"。
// 顺带把 UART 接收错误计数变化上报一次，方便上位机发现丢字节/溢出问题。
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

        last_wake_time += TELEMETRY_PERIOD_TICKS;
        if (osDelayUntil(last_wake_time) != osOK)
            Error_Handler();
    }
}
