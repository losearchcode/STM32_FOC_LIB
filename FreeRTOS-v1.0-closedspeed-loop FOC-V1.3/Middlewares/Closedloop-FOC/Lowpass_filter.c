#include "main.h"

Lowpass_Filter_t Lowpass_Vel_Filter_Para[LOWPASS_FILTER_NUM];
Lowpass_Filter_t Lowpass_Angle_Filter_Para[LOWPASS_FILTER_NUM];

void LowPassFilter_Init(Lowpass_Filter_t* filter,float time_constant)
{
	filter->Tf = time_constant;
	filter->y_prev = 0.0f;
	First_Time_Laod(&filter->Lowpass_Filter_Timecount);
}

float LowPassFilter_Operator(Lowpass_Filter_t* filter,float x)
{
    filter->Filter_Interval_Ts = Get_Interval_Timetick(&filter->Lowpass_Filter_Timecount);
    float dt = (filter->Filter_Interval_Ts)*1e-3f;

    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        filter->y_prev = x;
        return x;
    }

    float alpha = filter->Tf/(filter->Tf + dt);
    float y = alpha*filter->y_prev + (1.0f - alpha)*x;
    filter->y_prev = y;
    return y;
}

float LowPassFilter_Operator_(Lowpass_Filter_t* filter,float x)
{
    float y = 0.85*filter->y_prev + 0.15*x;
    filter->y_prev = y;
    return y;
}
