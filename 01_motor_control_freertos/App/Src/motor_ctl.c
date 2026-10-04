//
// Created by jedol on 2026/10/3.
//
#include "motor_ctl.h"
#include "motor.h"
#include "PID.h"
#include "cmsis_os2.h"

typedef enum {
    MOTOR_STATE_STOP = 0, //电机停
    MOTOR_STATE_SPEED,    //速度环：跟踪 motor_target_value（rpm，可负）
    MOTOR_STATE_POSITION  //位置环：相对当前角度走 motor_target_value 度（可负）
} MotorState_t;

// 数值解析结果：无内容（用默认值）/ 成功解析 / 非法
typedef enum {
    PARSE_EMPTY = 0,
    PARSE_NUMBER = 1,
    PARSE_INVALID = -1
} ParseResult_t;

static MotorState_t motor_state = MOTOR_STATE_STOP;
static uint8_t motor_state_changed;
static float motor_target_value;   // 与 motor_state 同锁保护的目标量
static osMutexId_t motor_state_mutex;

#define SPEED_TARGET 200.0f   // 裸 f/r 的默认转速（rpm）
#define POS_STEP     360.0f   // 裸 p/n 的默认相对步长（度）

// 解析可选正负号的十进制数值，允许一个小数点：
// 前后空白被忽略；无任何数字返回 PARSE_EMPTY；含非法字符返回 PARSE_INVALID。
static ParseResult_t MotorControl_ParseNumber(const char *text, float *value)
{
    uint8_t negative = 0;
    uint8_t has_digit = 0;
    float scale = 0.1f;
    float result = 0.0f;

    if ((text == NULL) || (value == NULL))
        return PARSE_INVALID;

    while ((*text == ' ') || (*text == '\t'))
        text++;

    if (*text == '\0')
        return PARSE_EMPTY;

    if ((*text == '+') || (*text == '-'))
    {
        negative = (uint8_t)(*text == '-');
        text++;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        result = result * 10.0f + (float)(*text - '0');
        has_digit = 1;
        text++;
    }

    if (*text == '.')
    {
        text++;
        while ((*text >= '0') && (*text <= '9'))
        {
            result += (float)(*text - '0') * scale;
            scale *= 0.1f;
            has_digit = 1;
            text++;
        }
    }

    if (!has_digit)
        return PARSE_INVALID;

    // 数值之后只允许空白，出现其它字符视为非法（如 f100x）
    while ((*text == ' ') || (*text == '\t'))
        text++;
    if (*text != '\0')
        return PARSE_INVALID;

    *value = negative ? -result : result;
    return PARSE_NUMBER;
}

// 初始化串级 PID 参数与控制状态，并创建保护状态的互斥量。
uint8_t MotorControl_Init(void)
{
    PidInit();
    motor_state = MOTOR_STATE_STOP;
    motor_state_changed = 0;
    motor_target_value = 0.0f;
    motor_state_mutex = osMutexNew(NULL);
    return (motor_state_mutex != NULL);
}

// 解析一整行命令并登记期望状态，返回解析结果
MotorControl_CommandResult_t MotorControl_HandleCommandLine(const char *line)
{
    MotorControl_CommandResult_t result = { MOTOR_CMD_NONE, 0.0f };
    MotorState_t requested_state;
    ParseResult_t parsed;
    float base_sign;
    float default_value;
    float value = 0.0f;
    const char *cursor;

    if (line == NULL)
    {
        result.kind = MOTOR_CMD_INVALID;
        return result;
    }

    // 跳过行首空白后定位命令字母
    cursor = line;
    while ((*cursor == ' ') || (*cursor == '\t'))
        cursor++;

    // 纯空白行：不处理也不回显
    if (*cursor == '\0')
        return result;

    switch (*cursor)
    {
        case 'f':
        case 'F':
            requested_state = MOTOR_STATE_SPEED;
            base_sign = 1.0f;
            default_value = SPEED_TARGET;
            break;
        case 'r':
        case 'R':
            requested_state = MOTOR_STATE_SPEED;
            base_sign = -1.0f;
            default_value = SPEED_TARGET;
            break;
        case 'p':
        case 'P':
            requested_state = MOTOR_STATE_POSITION;
            base_sign = 1.0f;
            default_value = POS_STEP;
            break;
        case 'n':
        case 'N':
            requested_state = MOTOR_STATE_POSITION;
            base_sign = -1.0f;
            default_value = POS_STEP;
            break;
        case 's':
        case 'S':
            // 停机不接受数值参数
            if (MotorControl_ParseNumber(cursor + 1, &value) != PARSE_EMPTY)
            {
                result.kind = MOTOR_CMD_INVALID;
                return result;
            }
            requested_state = MOTOR_STATE_STOP;
            base_sign = 0.0f;
            default_value = 0.0f;
            break;
        default:
            result.kind = MOTOR_CMD_INVALID;
            return result;
    }

    if (requested_state != MOTOR_STATE_STOP)
    {
        parsed = MotorControl_ParseNumber(cursor + 1, &value);
        //错误处理
        if (parsed == PARSE_INVALID)
        {
            result.kind = MOTOR_CMD_INVALID;
            return result;
        }

        // 未给数值时用默认量；给了数值时按 方向 × 数值 生效（如 f-100 -> -100）
        value = (parsed == PARSE_NUMBER) ?
                (base_sign * value) : (base_sign * default_value);
    }

    if ((motor_state_mutex == NULL) ||
        (osMutexAcquire(motor_state_mutex, osWaitForever) != osOK))
    {
        result.kind = MOTOR_CMD_INVALID;
        return result;
    }

    motor_state = requested_state;
    motor_target_value = value;
    motor_state_changed = 1;
    (void)osMutexRelease(motor_state_mutex);

    switch (requested_state)
    {
        case MOTOR_STATE_SPEED:
            result.kind = MOTOR_CMD_SPEED;
            result.value = value;
            break;
        case MOTOR_STATE_POSITION:
            result.kind = MOTOR_CMD_POSITION;
            result.value = value;
            break;
        default:
            result.kind = MOTOR_CMD_STOP;
            result.value = 0.0f;
            break;
    }

    return result;
}

// 将请求的电机状态应用到串级 PID 控制器上。
static void MotorControl_ApplyTarget(void)
{
    MotorState_t requested_state;
    float target_value;
    uint8_t state_changed;

    if ((motor_state_mutex == NULL) ||
        (osMutexAcquire(motor_state_mutex, osWaitForever) != osOK))
        return;

    state_changed = motor_state_changed;
    requested_state = motor_state;
    target_value = motor_target_value;
    motor_state_changed = 0;
    (void)osMutexRelease(motor_state_mutex);

    if (state_changed == 0)
        return;

    switch (requested_state)
    {
        // 速度环：直接跟踪带符号的目标转速
        case MOTOR_STATE_SPEED:
            Motor_1.State = PIDSPEED;
            Motor_1.SpeedExpected = target_value;
            break;

        // 位置环：相对当前角度走 target_value 度（可正可负）
        case MOTOR_STATE_POSITION:
            Motor_1.State = PIDPOSITION;
            Motor_1.PositionExpected =
                Motor_1.PositionMeasure + target_value;
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