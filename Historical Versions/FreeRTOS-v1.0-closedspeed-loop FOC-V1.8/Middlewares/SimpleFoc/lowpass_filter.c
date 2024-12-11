#include "lowpass_filter.h"
#include "main.h"
#include "BLDCMotor.h"

/******************************************************************************/
#define DEF_CURR_FILTER_Tf 0.005 //!< default currnet filter time constant
#define DEF_VEL_FILTER_Tf  0.005 //!< default velocity filter time constant
/******************************************************************************/
unsigned long lpf_vel_timestamp;
float y_vel_prev=0;
/******************************************************************************/
void LPF_init(uint8_t Motor_No)
{
	FOC_Para_M[Motor_No].LPF_current_q.Tf = 0.05;
	FOC_Para_M[Motor_No].LPF_current_q.y_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].LPF_current_q.Lowpass_Timecount);
	
	FOC_Para_M[Motor_No].LPF_current_d.Tf = 0.05;//Tf设置小一点，配合爬升斜率设置PID_velocity.output_ramp，速度切换更平稳；如果没有爬升模式的斜率限制，Tf太小电机容易抖动。
	FOC_Para_M[Motor_No].LPF_current_d.y_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].LPF_current_d.Lowpass_Timecount);
	
	FOC_Para_M[Motor_No].LPF_velocity.Tf = 0.0001;//Tf设置小一点，配合爬升斜率设置PID_velocity.output_ramp，速度切换更平稳；如果没有爬升模式的斜率限制，Tf太小电机容易抖动。
	FOC_Para_M[Motor_No].LPF_velocity.y_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].LPF_velocity.Lowpass_Timecount);
}


float LPFoperator(LowPassFilter_t * LPF,float x)
{
	float Ts_, alpha, y;
	Ts_ = (float)Get_Interval_Timetick(&LPF->Lowpass_Timecount)*1e-3f;
	
	if(Ts_ > 0.3f)   //时间过长，大概是程序刚启动初始化，直接返回
	{
		LPF->y_prev = x;
		return x;
	}
	
	alpha = LPF->Tf/(LPF->Tf	+ Ts_);
	
	
	y = alpha*LPF->y_prev + (1.0f - alpha)*x;
	LPF->y_prev = y;

	return y;

}

float LPF_velocity_(float x)
{
	float y = 0.9f*y_vel_prev + 0.1f*x;
	
	y_vel_prev=y;
	
	return y;
}
/******************************************************************************/
