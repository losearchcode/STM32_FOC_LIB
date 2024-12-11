#ifndef __FOC_PID_H_
#define __FOC_PID_H_
#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stm32f4xx_hal.h"
#include "Timecount.h" 

typedef struct {
		float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_vel_ramp; 
    float limit; 
    float error_vel_prev; //!< 最后的跟踪误差值
    float output_vel_prev;  //!< 最后一个 pid 输出值
    float integral_vel_prev;
		Interval_Timetick_t PID_Vel_Timecount;
}PID_Vel_t;

typedef struct {
		float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float error_ang_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
		Interval_Timetick_t PID_Angle_Timecount;
}PID_Angle_t;
	
	
/******************************************************************************/
float PID_angle(uint8_t Motor_No,float error);
float PID_velocity(uint8_t Motor_No,float error);
void PID_init(uint8_t Motor_No);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
	
#endif

