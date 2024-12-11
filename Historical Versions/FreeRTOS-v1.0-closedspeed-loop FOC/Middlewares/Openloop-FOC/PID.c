#include "main.h"

PID_Setting_Coefficient_t PID_Vel_Coefficient[PID_Controller_num]={
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // 第一个控制器的设置
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // 第二个控制器的设置
    };
PID_Setting_Coefficient_t PID_Angle_Coefficient[PID_Controller_num]={
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // 第一个控制器的设置
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // 第二个控制器的设置
    };


PID_Controller_t PID_Vel_Loop_Parameter[PID_Controller_num];
PID_Controller_t PID_Angle_Loop_Parameter[PID_Controller_num];

void PIDController_Init(PID_Controller_t *PID_control,float P, float I, float D, float ramp, float limit)
{
	PID_control->P = P;
	PID_control->I = I;
	PID_control->D = D;
	PID_control->output_ramp = ramp;	// PID控制器加速度限幅
	PID_control->limit = limit;         // PID控制器输出限幅
	PID_control->error_prev = 0.0f;
	PID_control->output_prev = 0.0f;
	PID_control->integral_prev = 0.0f;
	PID_control->PID_Interval_ts = 0;
	First_Time_Laod(&PID_control->PID_Controller_Timecount);
}

// PID 控制器函数
float PIDController_Operator(PID_Controller_t *PID_control,float error)
{
    // 计算两次循环中间的间隔时间
    PID_control->PID_Interval_ts = Get_Interval_Timetick(&PID_control->PID_Controller_Timecount);
    float Ts = (PID_control->PID_Interval_ts) * 1e-3f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
    // P环
    float proportional = PID_control->P * error;
    // Tustin 散点积分（I环）
    float integral = PID_control->integral_prev + PID_control->I*Ts*0.5f*(error + PID_control->error_prev);
    integral = _constrain(integral,-(PID_control->limit), PID_control->limit);
    // D环（微分环节）
    float derivative = PID_control->D*(error - PID_control->error_prev)/Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -(PID_control->limit), PID_control->limit);

    if(PID_control->output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - PID_control->output_prev)/Ts;
        if (output_rate > PID_control->output_ramp)
            output = PID_control->output_prev + PID_control->output_ramp*Ts;
        else if (output_rate < -(PID_control->output_ramp))
            output = PID_control->output_prev - PID_control->output_ramp*Ts;
    }
    // 保存值（为了下一次循环）
    PID_control->integral_prev = integral;
    PID_control->output_prev = output;
    PID_control->error_prev = error;
    return output;
}
