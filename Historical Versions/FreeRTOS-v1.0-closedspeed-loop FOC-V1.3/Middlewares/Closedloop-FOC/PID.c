#include "main.h"

PID_Setting_Coefficient_t PID_Vel_Coefficient[PID_Controller_num]={
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // ��һ��������������
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // �ڶ���������������
    };
PID_Setting_Coefficient_t PID_Angle_Coefficient[PID_Controller_num]={
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // ��һ��������������
        {2.0f, 0.0f, 0.0f, 100000.0f, 6.0f},  // �ڶ���������������
    };


PID_Controller_t PID_Vel_Loop_Parameter[PID_Controller_num];
PID_Controller_t PID_Angle_Loop_Parameter[PID_Controller_num];

void PIDController_Init(PID_Controller_t *PID_control,float P, float I, float D, float ramp, float limit)
{
	PID_control->P = P;
	PID_control->I = I;
	PID_control->D = D;
	PID_control->output_ramp = ramp;	// PID���������ٶ��޷�
	PID_control->limit = limit;         // PID����������޷�
	PID_control->error_prev = 0.0f;
	PID_control->output_prev = 0.0f;
	PID_control->integral_prev = 0.0f;
	PID_control->PID_Interval_ts = 0;
	First_Time_Laod(&PID_control->PID_Controller_Timecount);
}

// PID ����������
float PIDController_Operator(PID_Controller_t *PID_control,float error)
{
    // ��������ѭ���м�ļ��ʱ��
    PID_control->PID_Interval_ts = Get_Interval_Timetick(&PID_control->PID_Controller_Timecount);
    float Ts = (PID_control->PID_Interval_ts) * 1e-3f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
    // P��
    float proportional = PID_control->P * error;
    // Tustin ɢ����֣�I����
    float integral = PID_control->integral_prev + PID_control->I*Ts*0.5f*(error + PID_control->error_prev);
    integral = _constrain(integral,-(PID_control->limit), PID_control->limit);
    // D����΢�ֻ��ڣ�
    float derivative = PID_control->D*(error - PID_control->error_prev)/Ts;

    // ��P,I,D�����ļ���ֵ������
    float output = proportional + integral + derivative;
    output = _constrain(output, -(PID_control->limit), PID_control->limit);

    if(PID_control->output_ramp > 0){
        // ��PID�ı仯���ʽ�������
        float output_rate = (output - PID_control->output_prev)/Ts;
        if (output_rate > PID_control->output_ramp)
            output = PID_control->output_prev + PID_control->output_ramp*Ts;
        else if (output_rate < -(PID_control->output_ramp))
            output = PID_control->output_prev - PID_control->output_ramp*Ts;
    }
    // ����ֵ��Ϊ����һ��ѭ����
    PID_control->integral_prev = integral;
    PID_control->output_prev = output;
    PID_control->error_prev = error;
    return output;
}
