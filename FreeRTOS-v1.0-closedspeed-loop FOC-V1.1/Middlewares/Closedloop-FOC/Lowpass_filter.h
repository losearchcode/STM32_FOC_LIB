#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

#include "main.h"

#define LOWPASS_FILTER_NUM FOC_Motor_Num

typedef struct {
    float Tf; //!< ��ͨ�˲�ʱ�䳣��
		float y_prev; //!< ��һ��ѭ���еĹ��˺��ֵ
    Interval_Timetick_t Lowpass_Filter_Timecount;
		uint32_t Filter_Interval_Ts;  // ��һ��ʱ����
} Lowpass_Filter_t;


extern Lowpass_Filter_t Lowpass_Vel_Filter_Para[LOWPASS_FILTER_NUM];
extern Lowpass_Filter_t Lowpass_Angle_Filter_Para[LOWPASS_FILTER_NUM];


void LowPassFilter_Init(Lowpass_Filter_t* filter,float time_constant);
float LowPassFilter_Operator(Lowpass_Filter_t* filter,float x);


#endif /* __LOWPASS_FILTER_H */
