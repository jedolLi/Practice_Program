 #include "PID.h"

ClassicPidStructTypedef MotorPositionPid1,MotorSpeedPid1,MotorCurrentPid1;
ClassicPidStructTypedef MotorPositionPid2,MotorSpeedPid2,MotorCurrentPid2;
ClassicPidStructTypedef MotorPositionPid3,MotorSpeedPid3,MotorCurrentPid3;
ClassicPidStructTypedef MotorPositionPid4,MotorSpeedPid4,MotorCurrentPid4;

void MotorCurrentPidInit(void)
{
	MotorCurrentPid1.Kp = 1.0f;
	MotorCurrentPid1.Ki = 0.0f; 
	MotorCurrentPid1.Kd = 0.0f;
	MotorCurrentPid1.LimitOutput = 16000.0f;
	MotorCurrentPid1.LimitIntegral = 16000.0f;
	MotorCurrentPid1.Integral = 0;
	MotorCurrentPid1.PreError = 0;
	
	MotorCurrentPid2.Kp = 1.0f;
	MotorCurrentPid2.Ki = 0.01f; 
	MotorCurrentPid2.Kd = 0.0f;
	MotorCurrentPid2.LimitOutput = 16000.0f;
	MotorCurrentPid2.LimitIntegral = 16000.0f;
	MotorCurrentPid2.Integral = 0;
	MotorCurrentPid2.PreError = 0;

	MotorCurrentPid3.Kp = 1.0f;
	MotorCurrentPid3.Ki = 0.01f; 
	MotorCurrentPid3.Kd = 0.0f;
	MotorCurrentPid3.LimitOutput = 16000.0f;
	MotorCurrentPid3.LimitIntegral = 16000.0f;
	MotorCurrentPid3.Integral = 0;
	MotorCurrentPid3.PreError = 0;
	
	MotorCurrentPid4.Kp = 1.0f;
	MotorCurrentPid4.Ki = 0.01f; 
	MotorCurrentPid4.Kd = 0.0f;
	MotorCurrentPid4.LimitOutput = 16000.0f;
	MotorCurrentPid4.LimitIntegral = 16000.0f;
	MotorCurrentPid4.Integral = 0;
	MotorCurrentPid4.PreError = 0;
}

void MotorSpeedPidInit(void)
{
  MotorSpeedPid1.Kp = 40.0f;  
	MotorSpeedPid1.Ki =0.01f;     
	MotorSpeedPid1.Kd = 0.0002f;
	MotorSpeedPid1.LimitOutput = 5000.0f;
	MotorSpeedPid1.LimitIntegral = 5000.0f;
	MotorSpeedPid1.Integral = 0;
	MotorSpeedPid1.PreError = 0;
	
	MotorSpeedPid2.Kp = 2.0f;  
	MotorSpeedPid2.Ki =0.3f;     
	MotorSpeedPid2.Kd = 0.0f;
	MotorSpeedPid2.LimitOutput = 16000.0f;
	MotorSpeedPid2.LimitIntegral = 10000.0f;
	MotorSpeedPid2.Integral = 0;
	MotorSpeedPid2.PreError = 0;
	MotorSpeedPid2.SectionFlag = 0;
	MotorSpeedPid2.ErrorLine = 1000;
	MotorSpeedPid2.KpMax = 400;
	
	MotorSpeedPid3.Kp = 2.0f;  
	MotorSpeedPid3.Ki =0.3f;     
	MotorSpeedPid3.Kd = 0.0f;
	MotorSpeedPid3.LimitOutput = 16000.0f;
	MotorSpeedPid3.LimitIntegral = 10000.0f;
	MotorSpeedPid3.Integral = 0;
	MotorSpeedPid3.PreError = 0;
	MotorSpeedPid3.SectionFlag = 0;
	MotorSpeedPid3.ErrorLine = 1000;
	MotorSpeedPid3.KpMax = 400;
	
	MotorSpeedPid4.Kp = 2.0f;  
	MotorSpeedPid4.Ki =0.3f;     
	MotorSpeedPid4.Kd = 0.0f;
	MotorSpeedPid4.LimitOutput = 16000.0f;
	MotorSpeedPid4.LimitIntegral = 10000.0f;
	MotorSpeedPid4.Integral = 0;
	MotorSpeedPid4.PreError = 0;
	MotorSpeedPid4.SectionFlag = 0;
	MotorSpeedPid4.ErrorLine = 1000;
	MotorSpeedPid4.KpMax = 400;
}

void MotorPositionPidInit(void)
{
  MotorPositionPid1.Kp = 0.6f;
	MotorPositionPid1.Ki = 0.0f;   
	MotorPositionPid1.Kd = 0.0002f;
	MotorPositionPid1.LimitOutput = 16000.0f; 
	MotorPositionPid1.LimitIntegral = 2000.0f;
	MotorPositionPid1.Integral = 0;
	MotorPositionPid1.PreError = 0;
	MotorPositionPid1.PrePreError = 0;
	
	MotorPositionPid2.Kp = 2.3;   //2.2
	MotorPositionPid2.Ki = 0;
	MotorPositionPid2.Kd = 0.0001;
	MotorPositionPid2.LimitOutput = 16000;
	MotorPositionPid2.LimitIntegral = 2000;
	MotorPositionPid2.Integral = 0;
	MotorPositionPid2.PreError = 0;
	MotorPositionPid2.SectionFlag = 0;
	MotorPositionPid2.ErrorLine = 20;
	MotorPositionPid2.KpMax = 50;
	
	MotorPositionPid3.Kp = 2.3;   //2.2
	MotorPositionPid3.Ki = 0;
	MotorPositionPid3.Kd = 0.0001;
	MotorPositionPid3.LimitOutput = 16000; 
	MotorPositionPid3.LimitIntegral = 2000;
	MotorPositionPid3.Integral = 0;
	MotorPositionPid3.PreError = 0;
	MotorPositionPid3.SectionFlag = 0;
	MotorPositionPid3.ErrorLine = 20;
	MotorPositionPid3.KpMax = 50;
	
	MotorPositionPid4.Kp = 2.3;   //2.2
	MotorPositionPid4.Ki = 0;
	MotorPositionPid4.Kd = 0.0001;
	MotorPositionPid4.LimitOutput = 16000; 
	MotorPositionPid4.LimitIntegral = 2000;
	MotorPositionPid4.Integral = 0;
	MotorPositionPid4.PreError = 0;
	MotorPositionPid4.SectionFlag = 0;
	MotorPositionPid4.ErrorLine = 20;
	MotorPositionPid4.KpMax = 50;
}

// 依次初始化电流环、速度环、位置环参数（由 MotorControl_Init 调用）
void PidInit(void)
{
	MotorCurrentPidInit();
	MotorSpeedPidInit();	  
    MotorPositionPidInit();
}

// 位置式 PID 的一个 1ms 步进：P + 积分累加 + 差分微分，输出限幅。
// 积分项先累加再整体钳位（抗积分饱和），微分项用的是"误差差分"，
// 所以目标值突变时会有微分冲击，实际调参时 Kd 不宜给大。
float ClassicPidRegulate(float Reference, float PresentFeedback,ClassicPidStructTypedef *PID_Struct)
{
	float error;
	float error_inc;
	float pTerm;
	float iTerm;
	float dTerm;
	float dwAux;
	float output;
	/*error computation*/
	error = Reference - PresentFeedback;
	
	
	
	/*proportional term computation*/
	pTerm = error * PID_Struct->Kp;
	/*Integral term computation*/
	
	iTerm = error * PID_Struct->Ki;
	
	dwAux = PID_Struct->Integral + iTerm;
	/*limit integral*/
	if (dwAux > PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = PID_Struct->LimitIntegral;
	} else if (dwAux < -1*PID_Struct->LimitIntegral)
	{
		PID_Struct->Integral = -1*PID_Struct->LimitIntegral;
	} else
	{
	  PID_Struct->Integral = dwAux;
	}
	/*differential term computation*/
	
	error_inc = error - PID_Struct->PreError;
	dTerm = error_inc * PID_Struct->Kd;
	PID_Struct->PreError = error;

	output = pTerm + PID_Struct->Integral + dTerm;

	/*limit output*/
	if (output >= PID_Struct->LimitOutput)
	{
		return (PID_Struct->LimitOutput);
	} else if (output < -1.0f*PID_Struct->LimitOutput)
	{
		return (-1.0f*PID_Struct->LimitOutput);
	} else
	{
		return output;
	}
}
