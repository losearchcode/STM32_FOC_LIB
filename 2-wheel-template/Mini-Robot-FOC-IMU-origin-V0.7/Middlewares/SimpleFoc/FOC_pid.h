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
    float output_ramp; //!< ���ֱ���
    float limit; 
    float error_prev; //!< ���ĸ������ֵ
    float output_prev;  //!< ���һ�� pid ���ֵ
    float integral_prev;	//!< ���һ�����ַ���ֵ
		Interval_Timetick_t PID_Timecount;
}PIDController_t;

	
	
/******************************************************************************/
float PIDoperator(PIDController_t* PID,float error);
void PID_init(uint8_t Motor_No);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
	
#endif

