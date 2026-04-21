#include "pid.h"

/**
 * @brief PID计算函数
 * @warning error = Target - Measure
*/
float PID_Calculate(PID_Struct *PID, float Measure, float Target, float max_output)
{
	PID->Measure = Measure;
	PID->Target = Target;
	PID->Error = Target - Measure;

	PID->P_out = PID->Kp * PID->Error;
	PID->I_out += PID->Ki * ((PID->Error + PID->LastError)/2);//梯形积分
	PID->I_out = PID->I_out > (800 ? 800 : PID->I_out);//积分限幅
	PID->D_out = PID->Kd * (PID->LastMeasure - PID->Measure);//微分项使用测量值的变化率，减少微分环节对目标值突变的敏感性
	PID->Output = PID->P_out + PID->I_out + PID->D_out;

	PID->LastMeasure = PID->Measure;
	PID->LastLastError = PID->LastError;
	
	PID->LastError = PID->Error;

	if(PID->Output > max_output)
		{PID->Output = max_output;}
	else if(PID->Output < -max_output)
		{PID->Output = -max_output;}

	return PID->Output;
}

/**
 * @brief PID参数设置函数
*/
void PID_Set(PID_Struct *PID, float Kp, float Ki, float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
}


void PID_Clear (PID_Struct *PID)
{
	PID->I_out = 0;
	PID->LastError = 0;
	PID->Output = 0;
}
