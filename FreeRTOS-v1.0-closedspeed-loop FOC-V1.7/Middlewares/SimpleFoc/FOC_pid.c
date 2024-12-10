#include "FOC_pid.h"
#include "main.h"
#include "BLDCMotor.h"

/******************************************************************************/

/******************************************************************************/
void PID_init(uint8_t Motor_No)
{
	FOC_Para_M[Motor_No].PID_Vel_.P=0.068;  //0.1
	FOC_Para_M[Motor_No].PID_Vel_.I=0.037;    //2
	FOC_Para_M[Motor_No].PID_Vel_.D=0.0;    //2
	FOC_Para_M[Motor_No].PID_Vel_.output_vel_ramp=100;       //output derivative limit [volts/second]
	FOC_Para_M[Motor_No].PID_Vel_.integral_vel_prev=0;
	FOC_Para_M[Motor_No].PID_Vel_.error_vel_prev=0;
	FOC_Para_M[Motor_No].PID_Vel_.output_vel_prev=0;
	First_Time_Laod(&FOC_Para_M[Motor_No].PID_Vel_.PID_Vel_Timecount);
	
	FOC_Para_M[Motor_No].PID_Angle_.P=10;
	FOC_Para_M[Motor_No].PID_Angle_.I=0;
	FOC_Para_M[Motor_No].PID_Angle_.D=0.5;
	FOC_Para_M[Motor_No].PID_Angle_.error_ang_prev=0;
	FOC_Para_M[Motor_No].PID_Angle_.output_prev = 0;
	FOC_Para_M[Motor_No].PID_Angle_.integral_prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].PID_Angle_.PID_Angle_Timecount);
	
}
/******************************************************************************/
//just P&I is enough,no need D
float PID_velocity(uint8_t Motor_No,float error)
{
	float Ts;
	float proportional,integral,output;
	float output_rate;
	
	Ts = (float)Get_Interval_Timetick(&FOC_Para_M[Motor_No].PID_Vel_.PID_Vel_Timecount)/1e-3f;
  // quick fix for strange cases (micros overflow)
  if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f; 
	
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = FOC_Para_M[Motor_No].PID_Vel_.P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = FOC_Para_M[Motor_No].PID_Vel_.integral_vel_prev + FOC_Para_M[Motor_No].PID_Vel_.I*Ts*0.5f*(error + FOC_Para_M[Motor_No].PID_Vel_.error_vel_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit, FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit);
	
	// sum all the components
	output = proportional + integral;
	// antiwindup - limit the output variable
	output = _constrain(output, -FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit, FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit);
	
	// limit the acceleration by ramping the output
	output_rate = (output - FOC_Para_M[Motor_No].PID_Vel_.output_vel_prev)/Ts;
	if(output_rate > FOC_Para_M[Motor_No].PID_Vel_.output_vel_ramp)output = FOC_Para_M[Motor_No].PID_Vel_.output_vel_prev + FOC_Para_M[Motor_No].PID_Vel_.output_vel_ramp*Ts;
	else if(output_rate < -FOC_Para_M[Motor_No].PID_Vel_.output_vel_ramp)output = FOC_Para_M[Motor_No].PID_Vel_.output_vel_prev - FOC_Para_M[Motor_No].PID_Vel_.output_vel_ramp*Ts;
	
	// saving for the next pass
	FOC_Para_M[Motor_No].PID_Vel_.integral_vel_prev = integral;
	FOC_Para_M[Motor_No].PID_Vel_.output_vel_prev = output;
	FOC_Para_M[Motor_No].PID_Vel_.error_vel_prev = error;
	
	return output;
}
/******************************************************************************/
//P&D for angle_PID
float PID_angle(uint8_t Motor_No,float error)
{
	float Ts;
	float proportional,derivative,output;
	//float output_rate;
	
	
	Ts = (float)Get_Interval_Timetick(&FOC_Para_M[Motor_No].PID_Angle_.PID_Angle_Timecount)/1e-3f;
  // quick fix for strange cases (micros overflow)
  if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part 
	// u_p  = P *e(k)
	proportional = FOC_Para_M[Motor_No].PID_Angle_.P * error;
	// u_dk = D(ek - ek_1)/Ts
	derivative = FOC_Para_M[Motor_No].PID_Angle_.D*(error - FOC_Para_M[Motor_No].PID_Angle_.error_ang_prev)/Ts;
	
	output = proportional + derivative;
	output = _constrain(output, -FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit, FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit);
	
	// limit the acceleration by ramping the output
//	output_rate = (output - output_ang_prev)/Ts;
//	if(output_rate > output_ang_ramp)output = output_ang_prev + output_ang_ramp*Ts;
//	else if(output_rate < -output_ang_ramp)output = output_ang_prev - output_ang_ramp*Ts;
	
	// saving for the next pass
//	output_ang_prev = output;
	FOC_Para_M[Motor_No].PID_Angle_.error_ang_prev = error;
	
	return output;
}
/******************************************************************************/
/******************************************************************************/

