//
// Created by jedol on 2026/10/3.
//
#include "motor_ctl.h"
#include "motor.h"
#include "PID.h"
#include "cmsis_os2.h"

typedef enum {
    MOTOR_STATE_STOP = 0, //电机停
    MOTOR_STATE_FWD,
    MOTOR_STATE_REV,
    MOTOR_STATE_POS_FWD,
    MOTOR_STATE_POS_REV
} MotorState_t;

static MotorState_t motor_state = MOTOR_STATE_STOP;
static uint8_t motor_state_changed;
static osMutexId_t motor_state_mutex;

#define SPEED_TARGET 200.0f
#define POS_STEP     360.0f

// 初始化串级 PID 参数与控制状态，并创建保护状态的互斥量。
// 返回 0 表示互斥量创建失败（堆不足），调用方应视为致命错误。
uint8_t MotorControl_Init(void)
{
    PidInit();
    motor_state = MOTOR_STATE_STOP;
    motor_state_changed = 0;
    motor_state_mutex = osMutexNew(NULL);
    return (motor_state_mutex != NULL);
}

// 由 UartCmdTask 调用：只登记"期望的目标状态"，不直接改 Motor_1。
// 真正下发目标交给 1ms 控制任务，避免与 PID 运算并发改电机结构体。
void MotorControl_HandleCommand(uint8_t command)
{
    MotorState_t requested_state;

    // 回车换行只是串口终端的行结束符，不是控制指令
    if ((command == '\r') || (command == '\n'))
        return;

    switch (command)
    {
        case 'f':
        case 'F':
            requested_state = MOTOR_STATE_FWD;
            break;
        case 'r':
        case 'R':
            requested_state = MOTOR_STATE_REV;
            break;
        case 'p':
        case 'P':
            requested_state = MOTOR_STATE_POS_FWD;
            break;
        case 'n':
        case 'N':
            requested_state = MOTOR_STATE_POS_REV;
            break;
        case 's':
        case 'S':
            requested_state = MOTOR_STATE_STOP;
            break;
        default:
            return;
    }

    if ((motor_state_mutex == NULL) ||
        (osMutexAcquire(motor_state_mutex, osWaitForever) != osOK))
        return;

    motor_state = requested_state;
    motor_state_changed = 1;
    (void)osMutexRelease(motor_state_mutex);
}

// 把命令任务登记的目标套用到电机上。
// 临界区内只做"取出状态 + 清除变化标志"这一对读改写操作，
// 出临界区后再改电机目标：既保证标志原子性，又缩短持锁时间不拖慢 1ms 节拍。
static void MotorControl_ApplyTarget(void)
{
    MotorState_t requested_state;
    uint8_t state_changed;

    if ((motor_state_mutex == NULL) ||
        (osMutexAcquire(motor_state_mutex, osWaitForever) != osOK))
        return;

    state_changed = motor_state_changed;
    requested_state = motor_state;
    motor_state_changed = 0;
    (void)osMutexRelease(motor_state_mutex);

    if (state_changed == 0)
        return;

    switch (requested_state)
    {
        case MOTOR_STATE_FWD:
            Motor_1.State = PIDSPEED;
            Motor_1.SpeedExpected = SPEED_TARGET;
            break;

        case MOTOR_STATE_REV:
            Motor_1.State = PIDSPEED;
            Motor_1.SpeedExpected = -SPEED_TARGET;
            break;

        // 位置环目标按"当前位置 ± 一步"给出，所以重复按同一键会连续步进
        case MOTOR_STATE_POS_FWD:
            Motor_1.State = PIDPOSITION;
            Motor_1.PositionExpected =
                Motor_1.PositionMeasure + POS_STEP;
            break;

        case MOTOR_STATE_POS_REV:
            Motor_1.State = PIDPOSITION;
            Motor_1.PositionExpected =
                Motor_1.PositionMeasure - POS_STEP;
            break;

        // 停机要同时清积分项，否则下次启动会带着残留积分产生冲击
        case MOTOR_STATE_STOP:
        default:
            Motor_1.State = IDLE;
            Motor_1.SpeedExpected = 0;
            Motor_1.PWM = 0;
            MotorPositionPid1.Integral = 0;
            MotorSpeedPid1.Integral = 0;
            MotorCurrentPid1.Integral = 0;
            break;
    }
}

// 1ms 控制节拍：先套用新目标，再跑串级 PID 算出 PWM，最后经 CAN 下发
void MotorControl_Step(void)
{
    MotorControl_ApplyTarget();
    Motor(1);
    MotorUpdate((short)Motor_1.PWM, 0, 0, 0);
}