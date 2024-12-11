
#include "BLDCMotor.h"
#include "cmsis_os.h"

/************************************************
main中调用的接口函数都在当前文件中
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
extern float target;
/******************************************************************************/
long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int  pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;
/******************************************************************************/
int alignSensor(void);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
/******************************************************************************/
void Motor_init(void)
{
	printf("MOT: Init\r\n");
	
//	new_voltage_limit = current_limit * phase_resistance;
//	voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
	if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
	
	sensor_direction=UNKNOWN;
	
	FOC_EN(ON);
	printf("MOT: Enable driver.\r\n");
}
/******************************************************************************/
void Motor_initFOC(void)
{
	alignSensor();    //检测零点偏移量和极对数
	
	//added the shaft_angle update
	angle_prev=getAngle();  //getVelocity(),make sure velocity=0 after power on
	vTaskDelay(5);
	shaft_velocity = shaftVelocity();  //必须调用一次，进入主循环后速度为0
	vTaskDelay(5);
	shaft_angle = shaftAngle();// shaft angle
	if(controller==Type_angle)target=shaft_angle;//角度模式，以当前的角度为目标角度，进入主循环后电机静止
	
	vTaskDelay(200);
}
/******************************************************************************/
int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	
	printf("MOT: Align sensor.\r\n");
	
	// find natural direction
	// move one electrical revolution forward
	for(i=0; i<=500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setPhaseVoltage(voltage_sensor_align, 0,  angle);
		vTaskDelay(2);
	}
	mid_angle=getAngle();
	
	for(i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setPhaseVoltage(voltage_sensor_align, 0,  angle);
		vTaskDelay(2);
	}
	end_angle=getAngle();
	setPhaseVoltage(0, 0, 0);
	vTaskDelay(200);
	
	printf("mid_angle=%.4f\r\n",mid_angle);
	printf("end_angle=%.4f\r\n",end_angle);
	
	moved =  fabs(mid_angle - end_angle);
	if((mid_angle == end_angle)||(moved < 0.02))  //相等或者几乎没有动
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		FOC_EN(OFF);   //电机检测不正常，关闭驱动
		return 0;
	}
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		sensor_direction=CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		sensor_direction=CW;
	}
	
	
	printf("MOT: PP check: ");    //计算Pole_Pairs
	if( fabs(moved*pole_pairs - _2PI) > 0.5 )  // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		pole_pairs=_2PI/moved+0.5;     //浮点数转整形，四舍五入
		printf("%d\r\n",pole_pairs);
  }
	else
		printf("OK!\r\n");
	
	
	setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度
	vTaskDelay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
	vTaskDelay(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n",zero_electric_angle);
	
	setPhaseVoltage(0, 0, 0);
	vTaskDelay(200);
	
	return 1;
}
/******************************************************************************/
void loopFOC(void)
{
	if( controller==Type_angle_openloop || controller==Type_velocity_openloop ) return;
	
	shaft_angle = shaftAngle();// shaft angle
	electrical_angle = electricalAngle();// electrical angle - need shaftAngle to be called first
	
	switch(torque_controller)
	{
		case Type_voltage:  // no need to do anything really
			break;
		case Type_dc_current:
			break;
		case Type_foc_current:
			break;
		default:
			printf("MOT: no torque control selected!");
			break;
	}
	// set the phase voltage - FOC heart function :)
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
/******************************************************************************/
void move(float new_target)
{
	shaft_velocity = shaftVelocity();
	
	switch(controller)
	{
		case Type_torque:
			if(torque_controller==Type_voltage)voltage.q = new_target;  // if voltage torque control
		  else
				current_sp = new_target; // if current/foc_current torque control
			break;
		case Type_angle:
			// angle set point
      shaft_angle_sp = new_target;
      // calculate velocity set point
      shaft_velocity_sp = PID_angle( shaft_angle_sp - shaft_angle );
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if voltage torque control
      // if torque controlled through voltage  
      if(torque_controller == Type_voltage)
			{
				voltage.q = current_sp;
        voltage.d = 0;
      }
			break;
		case Type_velocity:
			// velocity set point
      shaft_velocity_sp = new_target;
      // calculate the torque command
      current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // if current/foc_current torque control
      // if torque controlled through voltage control 
      if(torque_controller == Type_voltage)
			{
        voltage.q = current_sp;  // use voltage if phase-resistance not provided
        voltage.d = 0;
      }
			break;
		case Type_velocity_openloop:
			// velocity control in open loop
      shaft_velocity_sp = new_target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
			break;
		case Type_angle_openloop:
			// angle control in open loop
      shaft_angle_sp = new_target;
      voltage.q = angleOpenloop(shaft_angle_sp); // returns the voltage that is set to the motor
      voltage.d = 0;
			break;
	}
}
/******************************************************************************/
// 设置PWM到控制器输出
void setPwm(float Ua, float Ub, float Uc) 
{
	  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //写入PWM到PWM 0 1 2 通道
	Set_TIM3_PWM_CCRx(PWM_1,dc_a);
	Set_TIM3_PWM_CCRx(PWM_2,dc_b);
	Set_TIM3_PWM_CCRx(PWM_3,dc_c);
}
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
	//	Sensor_update(FOC_Init_Para->Sensor_AS5600_); //更新传感器数值
  Uq=_constrain(Uq,-((voltage_power_supply)/2),(voltage_power_supply)/2);
//  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*_sin(angle_el); 
  Ubeta =   Uq*_cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + (voltage_power_supply)/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + (voltage_power_supply)/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + (voltage_power_supply)/2;
  setPwm(Ua,Ub,Uc);
//	float Uout;
//	uint32_t sector;
//	float T0,T1,T2;
//	float Ta,Tb,Tc;
//	
//	if(Ud) // only if Ud and Uq set 
//	{// _sqrt is an approx of sqrt (3-4% error)
//		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
//		// angle normalisation in between 0 and 2pi
//		// only necessary if using _sin and _cos - approximation functions
//		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
//	}
//	else
//	{// only Uq available - no need for atan2 and sqrt
//		Uout = Uq / voltage_power_supply;
//		// angle normalisation in between 0 and 2pi
//		// only necessary if using _sin and _cos - approximation functions
//		angle_el = _normalizeAngle(angle_el + _PI_2);
//	}
//	if(Uout> 0.577)Uout= 0.577;
//	if(Uout<-0.577)Uout=-0.577;
//	
//	sector = (angle_el / _PI_3) + 1;
//	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
//	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
//	T0 = 1 - T1 - T2;
//	
//	// calculate the duty cycles(times)
//	switch(sector)
//	{
//		case 1:
//			Ta = T1 + T2 + T0/2;
//			Tb = T2 + T0/2;
//			Tc = T0/2;
//			break;
//		case 2:
//			Ta = T1 +  T0/2;
//			Tb = T1 + T2 + T0/2;
//			Tc = T0/2;
//			break;
//		case 3:
//			Ta = T0/2;
//			Tb = T1 + T2 + T0/2;
//			Tc = T2 + T0/2;
//			break;
//		case 4:
//			Ta = T0/2;
//			Tb = T1+ T0/2;
//			Tc = T1 + T2 + T0/2;
//			break;
//		case 5:
//			Ta = T2 + T0/2;
//			Tb = T0/2;
//			Tc = T1 + T2 + T0/2;
//			break;
//		case 6:
//			Ta = T1 + T2 + T0/2;
//			Tb = T0/2;
//			Tc = T1 + T0/2;
//			break;
//		default:  // possible error state
//			Ta = 0;
//			Tb = 0;
//			Tc = 0;
//	}
//	
//	Set_TIM3_PWM_CCRx(PWM_1,Ta);
//	Set_TIM3_PWM_CCRx(PWM_2,Tb);
//	Set_TIM3_PWM_CCRx(PWM_3,Tc);
}
/******************************************************************************/
float velocityOpenloop(float target_velocity)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = HAL_GetTick(); //_micros();
	if(now_us>=open_loop_timestamp)Ts = (float)(now_us - open_loop_timestamp)/1e-3;
	else
		Ts = (float)(0xFFFFFF - open_loop_timestamp + now_us)/1e-3;
	open_loop_timestamp=now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 
	
	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
	return Uq;
}
/******************************************************************************/
float angleOpenloop(float target_angle)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = HAL_GetTick(); //_micros();
	if(now_us>=open_loop_timestamp)Ts = (float)(now_us - open_loop_timestamp)/1e-3;
	else
		Ts = (float)(0xFFFFFF - open_loop_timestamp + now_us)/1e-3;
	open_loop_timestamp=now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(fabs( target_angle - shaft_angle ) > velocity_limit*Ts)
	{
    shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
    //shaft_velocity = velocity_limit;
  }
	else
	{
    shaft_angle = target_angle;
    //shaft_velocity = 0;
  }
	
	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
  return Uq;
}
/******************************************************************************/



