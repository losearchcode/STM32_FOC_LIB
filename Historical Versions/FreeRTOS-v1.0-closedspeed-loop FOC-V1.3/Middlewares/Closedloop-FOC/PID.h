#ifndef __P_I_D_H
#define __P_I_D_H

#include "main.h"

#define PID_Controller_num FOC_Motor_Num

typedef struct {
		float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float ramp; 
    float limit; 
}PID_Setting_Coefficient_t;

typedef struct {
		float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_ramp; 
    float limit; 
    float error_prev; //!< 最后的跟踪误差值
    float output_prev;  //!< 最后一个 pid 输出值
    float integral_prev; //!< 最后一个积分分量值
		Interval_Timetick_t PID_Controller_Timecount;
    uint32_t PID_Interval_ts;  // 用于计算速度的上一次时间戳
}PID_Controller_t;

extern PID_Setting_Coefficient_t PID_Vel_Coefficient[PID_Controller_num];
extern PID_Setting_Coefficient_t PID_Angle_Coefficient[PID_Controller_num];
extern PID_Controller_t PID_Vel_Loop_Parameter[PID_Controller_num];
extern PID_Controller_t PID_Angle_Loop_Parameter[PID_Controller_num];

void PIDController_Init(PID_Controller_t *PID_control,float P, float I, float D, float ramp, float limit);
float PIDController_Operator(PID_Controller_t *PID_control,float error);


#endif /* __P_I_D_H */
