#include "FOC_pid.h"
#include "main.h"
#include "BLDCMotor.h"

/******************************************************************************/
/*
低速：
P 0.105
I 0.048
D 0.0005

0.115
0.055
0.0005
*/
/******************************************************************************/
void PID_init(uint8_t Motor_No)
{
	FOC_Para_M[Motor_No].PID_Vel_.P=0.081;  //0.1
	FOC_Para_M[Motor_No].PID_Vel_.I=0.041;    //2
	FOC_Para_M[Motor_No].PID_Vel_.D=0.0;    //2
	FOC_Para_M[Motor_No].PID_Vel_.output_ramp=100;       //output derivative limit [volts/second]
	FOC_Para_M[Motor_No].PID_Vel_.limit=FOC_Para_M[0].Motor_Properties_.Voltage_Limit;        //Motor_init()函数已经对limit初始化，此处无需处理
	FOC_Para_M[Motor_No].PID_Vel_.error_prev=0;
	FOC_Para_M[Motor_No].PID_Vel_.integral_prev=0;
	FOC_Para_M[Motor_No].PID_Vel_.output_prev=0;
	First_Time_Laod(&FOC_Para_M[Motor_No].PID_Vel_.PID_Timecount);
	
	FOC_Para_M[Motor_No].PID_Angle_.P=10;
	FOC_Para_M[Motor_No].PID_Angle_.I=0;
	FOC_Para_M[Motor_No].PID_Angle_.D=0.5;
	FOC_Para_M[Motor_No].PID_Angle_.output_ramp=0;
	FOC_Para_M[Motor_No].PID_Angle_.limit=FOC_Para_M[0].Motor_Properties_.Velocity_Limit;        //Motor_init()函数已经对limit初始化，此处无需处理
	FOC_Para_M[Motor_No].PID_Angle_.error_prev=0;
	FOC_Para_M[Motor_No].PID_Angle_.output_prev = 0;
	FOC_Para_M[Motor_No].PID_Angle_.integral_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].PID_Angle_.PID_Timecount);
	
	FOC_Para_M[Motor_No].PID_current_q_.P=0.5;  //航模电机，速度闭环，不能大于1，否则容易失控
	FOC_Para_M[Motor_No].PID_current_q_.I=0;    //电流环I参数不太好调试，只用P参数也可以
	FOC_Para_M[Motor_No].PID_current_q_.D=0;
	FOC_Para_M[Motor_No].PID_current_q_.output_ramp=0;
	FOC_Para_M[Motor_No].PID_current_q_.limit=FOC_Para_M[0].Motor_Properties_.Current_Limit;
	FOC_Para_M[Motor_No].PID_current_q_.error_prev=0;
	FOC_Para_M[Motor_No].PID_current_q_.output_prev=0;
	FOC_Para_M[Motor_No].PID_current_q_.integral_prev=0;
	First_Time_Laod(&FOC_Para_M[Motor_No].PID_current_q_.PID_Timecount);
	
	FOC_Para_M[Motor_No].PID_current_d_.P=0.5;  //0.5
	FOC_Para_M[Motor_No].PID_current_d_.I=0;
	FOC_Para_M[Motor_No].PID_current_d_.D=0;
	FOC_Para_M[Motor_No].PID_current_d_.output_ramp=0;
	FOC_Para_M[Motor_No].PID_current_d_.limit=FOC_Para_M[0].Motor_Properties_.Current_Limit;
	FOC_Para_M[Motor_No].PID_current_d_.error_prev=0;
	FOC_Para_M[Motor_No].PID_current_d_.output_prev=0;
	FOC_Para_M[Motor_No].PID_current_d_.integral_prev=0;
	First_Time_Laod(&FOC_Para_M[Motor_No].PID_current_d_.PID_Timecount);
	
	
}
/******************************************************************************/
//just P&I is enough,no need D
float PIDoperator(PIDController_t* PID,float error)
{
	float Ts;
	float proportional,integral,derivative,output;
	float output_rate;

	
	Ts = (float)Get_Interval_Timetick(&PID->PID_Timecount)*1e-3f;
	if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	proportional = PID->P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = PID->integral_prev + PID->I*Ts*0.5f*(error + PID->error_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -PID->limit, PID->limit);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	derivative = PID->D*(error - PID->error_prev)/Ts;
	
	// sum all the components
	output = proportional + integral + derivative;
	// antiwindup - limit the output variable
	output = _constrain(output, -PID->limit, PID->limit);
	
	// if output ramp defined
	if(PID->output_ramp > 0)
	{
		// limit the acceleration by ramping the output
		output_rate = (output - PID->output_prev)/Ts;
		if(output_rate > PID->output_ramp)output = PID->output_prev + PID->output_ramp*Ts;
		else if(output_rate < -PID->output_ramp)output = PID->output_prev - PID->output_ramp*Ts;
	}
	
	// saving for the next pass
	PID->integral_prev = integral;
	PID->output_prev = output;
	PID->error_prev = error;
	
	return output;
}

/******************************************************************************/

