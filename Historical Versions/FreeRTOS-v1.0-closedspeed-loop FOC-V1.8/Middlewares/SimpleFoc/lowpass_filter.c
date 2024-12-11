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
	
	FOC_Para_M[Motor_No].LPF_current_d.Tf = 0.05;//Tf����Сһ�㣬�������б������PID_velocity.output_ramp���ٶ��л���ƽ�ȣ����û������ģʽ��б�����ƣ�Tf̫С������׶�����
	FOC_Para_M[Motor_No].LPF_current_d.y_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].LPF_current_d.Lowpass_Timecount);
	
	FOC_Para_M[Motor_No].LPF_velocity.Tf = 0.0001;//Tf����Сһ�㣬�������б������PID_velocity.output_ramp���ٶ��л���ƽ�ȣ����û������ģʽ��б�����ƣ�Tf̫С������׶�����
	FOC_Para_M[Motor_No].LPF_velocity.y_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].LPF_velocity.Lowpass_Timecount);
}


float LPFoperator(LowPassFilter_t * LPF,float x)
{
	float Ts_, alpha, y;
	Ts_ = (float)Get_Interval_Timetick(&LPF->Lowpass_Timecount)*1e-3f;
	
	if(Ts_ > 0.3f)   //ʱ�����������ǳ����������ʼ����ֱ�ӷ���
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
