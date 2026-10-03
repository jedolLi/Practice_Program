#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h"
#include <math.h>
#include "stdio.h"
#include "main.h"

#define Motor_1_ID	0x201
#define Motor_2_ID  0x202
#define Motor_3_ID  0x203
#define Motor_4_ID  0x204
#define Motor1_LandPos   53.51f   // 电机 1 的落点角度（度），过流保护回退判据

// 电机工作状态机：State 决定 Motor() 里跑哪一级环
typedef enum
{
	IDLE,           // 停机，PWM 输出 0
	PIDSPEED,       // 速度环：直接跟踪 SpeedExpected
	PIDPOSITION,    // 位置环：位置->速度->电流三环级联
	MOTOR_ERROR,
	MOTOR_PWM,      // 开环：直接给 PWM
	MOTOR_CURRENT,  // 单电流环
	MOTOR_BUFFER,   // 位置环 + 过流限流回退
} DriverState;

// 单个电机的全部状态。Expected* 由控制任务写，Measure* 由 CAN 中断写，
// PosPre/PosNow 保存两帧机械角用于计算增量。
typedef struct
{
	float PositionExpected;   // 目标位置（度，多圈累计）
	float PositionMeasure;    // 实测位置（度，多圈累计）
	float SpeedExpected;      // 目标转速（输出轴 rpm）
	float SpeedMeasure;       // 实测转速（输出轴 rpm）

	float CurrentExpected;    // 目标电流（PID 中间量，非物理单位）
	float CurrentMeasure;     // 实测转矩电流（原始值）

	short PosPre;             // 上一帧机械角 0~8191
	short PosNow;             // 当前帧机械角 0~8191

	int32_t PWM;              // 输出给电调的电流指令，范围 ±16000
	DriverState State;

} MotorTypeDef;

extern volatile MotorTypeDef Motor_1;
extern volatile MotorTypeDef Motor_2;
extern volatile MotorTypeDef Motor_3;
extern volatile MotorTypeDef Motor_4;

void Motor(int num);
void MotorUpdate(short I1, short I2, short I3, short I4);
float Get_RM3508_Distance(MotorTypeDef Motor);

#endif
