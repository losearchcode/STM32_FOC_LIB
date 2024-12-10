#ifndef __FOC_PID_H_
#define __FOC_PID_H_
#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stm32f4xx_hal.h"
#include "Timecount.h" 

typedef struct {
		float P; //!< ��������(P������)
    float I; //!< �������棨I�����棩
    float D; //!< ΢�����棨D�����棩
    float output_vel_ramp; 
    float limit; 
    float error_vel_prev; //!< ���ĸ������ֵ
    float output_vel_prev;  //!< ���һ�� pid ���ֵ
    float integral_vel_prev;
		Interval_Timetick_t PID_Vel_Timecount;
}PID_Vel_t;

typedef struct {
		float P; //!< ��������(P������)
    float I; //!< �������棨I�����棩
    float D; //!< ΢�����棨D�����棩
    float error_ang_prev; //!< ���ĸ������ֵ
    float output_prev;  //!< ���һ�� pid ���ֵ
    float integral_prev; //!< ���һ�����ַ���ֵ
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

