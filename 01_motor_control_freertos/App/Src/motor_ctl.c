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
uint8_t MotorControl_Init(void)
{
    PidInit();
    motor_state = MOTOR_STATE_STOP;
    motor_state_changed = 0;
    motor_state_mutex = osMutexNew(NULL);
    return (motor_state_mutex != NULL);
}

// 处理串口命令，设置电机目标状态。
void MotorControl_HandleCommand(uint8_t command)
{
    MotorState_t requested_state;

    // 忽略回车换行
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

// 将请求的电机状态应用到串级 PID 控制器上。
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

        // 停机要同时清积分项
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

// 在 FreeRTOS 任务中调用此函数以执行电机控制循环。
void MotorControl_Step(void)
{
    MotorControl_ApplyTarget();
    Motor(1);
    MotorUpdate((short)Motor_1.PWM, 0, 0, 0);
}