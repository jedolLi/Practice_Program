#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
     float Kp;
     float Ki;
     float Kd;
     float LimitOutput;    // 输出限幅（对称，±LimitOutput）
     float LimitIntegral;  // 积分项限幅，抗积分饱和
     float Integral;       // 积分累加值（状态量，停机时需手动清零）
     float PreError;       // 上一拍误差，用于算微分
     float PrePreError;
     int SectionFlag;
     float KpMax;
     float ErrorLine;
} ClassicPidStructTypedef;

// 4 个电机各一套位置/速度/电流环参数
extern ClassicPidStructTypedef MotorPositionPid1, MotorSpeedPid1, MotorCurrentPid1;
extern ClassicPidStructTypedef MotorPositionPid2, MotorSpeedPid2, MotorCurrentPid2;
extern ClassicPidStructTypedef MotorPositionPid3, MotorSpeedPid3, MotorCurrentPid3;
extern ClassicPidStructTypedef MotorPositionPid4, MotorSpeedPid4, MotorCurrentPid4;

void MotorCurrentPidInit(void);
void MotorSpeedPidInit(void);
void MotorPositionPidInit(void);

// 一次性初始化三环参数，由 MotorControl_Init 调用
void PidInit(void);

// PID 单步计算：传入目标值与反馈值，返回限幅后的输出
float ClassicPidRegulate(float Reference, float PresentFeedback, ClassicPidStructTypedef *PidStruct);

#ifdef __cplusplus
}
#endif

#endif
