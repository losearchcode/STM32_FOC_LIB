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

//float LPF_velocity(float x)
//{
//	unsigned long now_us;
//	float Ts, alpha, y;
//	
//	
//	now_us = HAL_GetTick(); //_micros();
//	if(now_us>=lpf_vel_timestamp)Ts = (float)(now_us - lpf_vel_timestamp)/1e-3;
//	else
//		Ts = (float)(0xFFFFFF - lpf_vel_timestamp + now_us)/1e-3;
//	lpf_vel_timestamp=now_us;  //save timestamp for next call
//	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f; 
//	
//	alpha = DEF_VEL_FILTER_Tf/(DEF_VEL_FILTER_Tf + Ts);
//	y = alpha*y_vel_prev + (1.0f - alpha)*x;
//	y_vel_prev = y;
//	
//	return y;
//}

float LPF_velocity(float x)
{
	float y = 0.9f*y_vel_prev + 0.1f*x;
	
	y_vel_prev=y;
	
	return y;
}
/******************************************************************************/
