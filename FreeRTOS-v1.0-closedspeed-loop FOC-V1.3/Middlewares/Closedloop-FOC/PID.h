#ifndef __P_I_D_H
#define __P_I_D_H

#include "main.h"

#define PID_Controller_num FOC_Motor_Num

typedef struct {
		float P; //!< ��������(P������)
    float I; //!< �������棨I�����棩
    float D; //!< ΢�����棨D�����棩
    float ramp; 
    float limit; 
}PID_Setting_Coefficient_t;

typedef struct {
		float P; //!< ��������(P������)
    float I; //!< �������棨I�����棩
    float D; //!< ΢�����棨D�����棩
    float output_ramp; 
    float limit; 
    float error_prev; //!< ���ĸ������ֵ
    float output_prev;  //!< ���һ�� pid ���ֵ
    float integral_prev; //!< ���һ�����ַ���ֵ
		Interval_Timetick_t PID_Controller_Timecount;
    uint32_t PID_Interval_ts;  // ���ڼ����ٶȵ���һ��ʱ���
}PID_Controller_t;

extern PID_Setting_Coefficient_t PID_Vel_Coefficient[PID_Controller_num];
extern PID_Setting_Coefficient_t PID_Angle_Coefficient[PID_Controller_num];
extern PID_Controller_t PID_Vel_Loop_Parameter[PID_Controller_num];
extern PID_Controller_t PID_Angle_Loop_Parameter[PID_Controller_num];

void PIDController_Init(PID_Controller_t *PID_control,float P, float I, float D, float ramp, float limit);
float PIDController_Operator(PID_Controller_t *PID_control,float error);


#endif /* __P_I_D_H */
