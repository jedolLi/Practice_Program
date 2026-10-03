#include "motor.h"
#include "main.h"
#include "can.h"
#include "PID.h"

#define CurrentLimit 2000
#define CurrentLine  1700

volatile MotorTypeDef Motor_1;
volatile MotorTypeDef Motor_2;
volatile MotorTypeDef Motor_3;
volatile MotorTypeDef Motor_4;

// 过流保护标志：置 1 表示已进入限流状态（见 MOTOR_BUFFER 分支）
int CurrentFlag1 = 0;

// 按当前状态跑串级 PID，结果写回对应电机。
//
// 这里刻意先把 Motor_x 拷进局部变量再运算、结束时只写回控制量：
// Motor_x 同时被 CAN 中断更新（SpeedMeasure / PositionMeasure / CurrentMeasure）。
// 若像以前那样整体拷回，会把中断刚写进去的最新反馈值用这份旧副本覆盖掉，
// 导致转速与位置反馈永远滞后甚至丢失。
void Motor(int num)
{
	volatile MotorTypeDef *motor_ref;
	MotorTypeDef motor;
	switch (num)
	{
		case 1: motor_ref = &Motor_1; break;
		case 2: motor_ref = &Motor_2; break;
		case 3: motor_ref = &Motor_3; break;
		case 4: motor_ref = &Motor_4; break;
		default: return;
	}
	motor = *motor_ref;

	// 位置环 -> 速度环 -> 电流环依次串下来，靠"故意不加 break"实现级联：
	// 上环的输出直接作为下环的目标，最后一级算出 PWM。
	switch (motor.State)
	{
	case PIDPOSITION:
		motor.SpeedExpected = ClassicPidRegulate(motor.PositionExpected, motor.PositionMeasure, &MotorPositionPid1);
		CurrentFlag1 = 0;

	case PIDSPEED:
		motor.CurrentExpected = ClassicPidRegulate(motor.SpeedExpected, motor.SpeedMeasure, &MotorSpeedPid1);

	case MOTOR_CURRENT:
		motor.PWM = (int32_t)ClassicPidRegulate(motor.CurrentExpected, motor.CurrentMeasure, &MotorCurrentPid1);
		break;

	// 带回落的串级控制：正常时跑位置->速度->电流三环；
	// 电流超过 CurrentLimit 就切到限流模式，把电流环目标钳到 CurrentLine 压住过流；
	// 等位置回到目标附近（<15 度）再自动解除限流，恢复三环控制。
	case MOTOR_BUFFER:
		if (motor.CurrentMeasure >= -1 * CurrentLimit && CurrentFlag1 == 0)
		{
			motor.SpeedExpected   = ClassicPidRegulate(motor.PositionExpected, motor.PositionMeasure, &MotorPositionPid1);
			motor.CurrentExpected = ClassicPidRegulate(motor.SpeedExpected, motor.SpeedMeasure, &MotorSpeedPid1);
			motor.PWM             = (int32_t)ClassicPidRegulate(motor.CurrentExpected, motor.CurrentMeasure, &MotorCurrentPid1);
		}
		else
		{
			motor.PWM = (int32_t)ClassicPidRegulate(-1 * CurrentLine, motor.CurrentMeasure, &MotorCurrentPid1);
			CurrentFlag1 = 1;
		}

		if (fabs(motor.PositionMeasure - Motor1_LandPos) < 15)
		{
			CurrentFlag1 = 0;
		}
		break;

	case MOTOR_PWM:
		break;

	default:
		motor.PWM = 0;
		break;
	}

	// 只回写控制量，绝不整体覆盖，保护中断写入的反馈值
	motor_ref->SpeedExpected = motor.SpeedExpected;
	motor_ref->CurrentExpected = motor.CurrentExpected;
	motor_ref->PWM = motor.PWM;
}

// 打包 4 路电流到 0x200 报文
void MotorUpdate(short I1, short I2, short I3, short I4)
{
	uint8_t Data[8];
	Data[0] = (uint8_t)(I1 >> 8);
	Data[1] = (uint8_t)(I1 & 0xff);
	Data[2] = (uint8_t)(I2 >> 8);
	Data[3] = (uint8_t)(I2 & 0xff);
	Data[4] = (uint8_t)(I3 >> 8);
	Data[5] = (uint8_t)(I3 & 0xff);
	Data[6] = (uint8_t)(I4 >> 8);
	Data[7] = (uint8_t)(I4 & 0xff);
	CAN_SendMessage(0x200, Data, 8);
}

// 计算相对上一帧的角度增量（度，输出轴）。
// RM3508 编码器是 0~8191 的单圈计数，跨零点时原始差值会突变约 ±8192。
// 这里以"半圈"为阈值判断是否绕圈：超过 4000 就补/减一圈，还原真实增量。
float Get_RM3508_Distance(MotorTypeDef Motor)
{
	int Distance = Motor.PosNow - Motor.PosPre;
	if (fabs((float)Distance) > 4000)
		Distance = Distance - Distance / fabs((float)Distance) * 8192;
	return ((float)Distance * 360.0f / 19.0f / 8192.0f);
}
