#include "BLDCMotor.h"
#include "main.h"
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
FOC_Para_All_t  FOC_Para_M[FOC_Motor_Num];

/******************************************************************************/

int alignSensor(uint8_t Motor_No);
float velocityOpenloop(uint8_t Motor_No,float target_velocity);
float angleOpenloop(uint8_t Motor_No,float target_angle);

/******************************************************************************/
void Motor_init(uint8_t Motor_No)
{
	printf("MOT: Init\r\n");
	
//	new_voltage_limit = current_limit * phase_resistance;
//	voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
	if(FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Sensor_Align > FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit) 
			FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Sensor_Align = FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit;
	/**************************       电机属性设置      **************************************/	
	FOC_Para_M[Motor_No].Motor_Properties_.FOC_No_Num = Motor_No;
	FOC_Para_M[Motor_No].Motor_Properties_.TimHandle = &htim3;
	FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply = 12 ; 	//V，电源电压
	FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit = 6;						//V，最大值需小于12/1.732=6.9
	FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Sensor_Align = 2;		//V  重要参数，航模电机大功率0.5-1，云台电机小功率2-3
	FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs = 7;				//极对数：7
	FOC_Para_M[Motor_No].Motor_Properties_.Sensor_DIR=UNKNOWN;


	/**************************       电机FOC状态设置      **************************************/		
	FOC_Para_M[Motor_No].Motor_State_.torque_controller = Type_voltage;	//当前只有电压模式
	FOC_Para_M[Motor_No].Motor_State_.controller = Type_velocity;				//速度环控制
	
	FOC_Para_M[Motor_No].Motor_State_.shaft_angle = 0;
	FOC_Para_M[Motor_No].Motor_State_.electrical_angle = 0;
	FOC_Para_M[Motor_No].Motor_State_.shaft_velocity =0;
	FOC_Para_M[Motor_No].Motor_State_.current_sp = 0;
	FOC_Para_M[Motor_No].Motor_State_.shaft_angle_sp =0;
	FOC_Para_M[Motor_No].Motor_State_.sensor_offset =0;
	FOC_Para_M[Motor_No].Motor_State_.zero_electric_angle =0;
	FOC_Para_M[Motor_No].Motor_State_.voltage.d = 0;
	FOC_Para_M[Motor_No].Motor_State_.voltage.q =0;
	FOC_Para_M[Motor_No].Motor_State_.current.d =0;
	FOC_Para_M[Motor_No].Motor_State_.current.q =0;
	
	FOC_EN(ON);
	printf("MOT: Enable driver.\r\n");
}
/******************************************************************************/
void Motor_initFOC(uint8_t Motor_No)
{
	alignSensor(Motor_No);    //检测零点偏移量和极对数
	
	//added the shaft_angle update
	FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev = getAngle(Motor_No);  //getVelocity(),make sure velocity=0 after power on
	vTaskDelay(5);
	FOC_Para_M[Motor_No].Motor_State_.shaft_velocity = shaftVelocity(Motor_No);  //必须调用一次，进入主循环后速度为0
	vTaskDelay(5);
	FOC_Para_M[Motor_No].Motor_State_.shaft_angle = shaftAngle(Motor_No);// shaft angle
	if(FOC_Para_M[Motor_No].Motor_State_.controller==Type_angle) target=FOC_Para_M[Motor_No].Motor_State_.shaft_angle;//角度模式，以当前的角度为目标角度，进入主循环后电机静止
	
	vTaskDelay(200);
}
/******************************************************************************/
int alignSensor(uint8_t Motor_No)
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
		angle = _3PI_2 + _2PI * i / 500.0f;
		setPhaseVoltage(Motor_No,FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Sensor_Align, 0,  angle);
		vTaskDelay(2);
	}
	mid_angle=getAngle(Motor_No);
	
	for(i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0f ;
		setPhaseVoltage(Motor_No,FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Sensor_Align, 0,  angle);
		vTaskDelay(2);
	}
	end_angle=getAngle(Motor_No);
	setPhaseVoltage(Motor_No,0, 0, 0);
	vTaskDelay(200);
	
	printf("mid_angle=%.4f\r\n",mid_angle);
	printf("end_angle=%.4f\r\n",end_angle);
	
	moved =  fabs(mid_angle - end_angle);
	if((mid_angle == end_angle)||(moved < 0.02f))  //相等或者几乎没有动
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		FOC_EN(OFF);   //电机检测不正常，关闭驱动
		return 0;
	}
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		FOC_Para_M[Motor_No].Motor_Properties_.Sensor_DIR=CCW;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		FOC_Para_M[Motor_No].Motor_Properties_.Sensor_DIR=CW;
	}
	
	
	printf("MOT: PP check: ");    //计算Pole_Pairs
	if( fabs(moved*FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs - _2PI) > 0.5f )  // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs = _2PI/moved+0.5f;     //浮点数转整形，四舍五入
		printf("%d\r\n",FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs);
  }
	else
		printf("OK!\r\n");
	
	
	setPhaseVoltage(Motor_No,FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Sensor_Align, 0,  _3PI_2);  //计算零点偏移角度
	vTaskDelay(700);
	FOC_Para_M[Motor_No].Motor_State_.zero_electric_angle = _normalizeAngle(_electricalAngle(FOC_Para_M[Motor_No].Motor_Properties_.Sensor_DIR*
																													getAngle(Motor_No), FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs));
	vTaskDelay(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n",FOC_Para_M[Motor_No].Motor_State_.zero_electric_angle);
	
	setPhaseVoltage(Motor_No,0, 0, 0);
	vTaskDelay(200);
	
	printf("MOT torque_controller: %d\n",FOC_Para_M[0].Motor_State_.torque_controller);
	printf("MOT controller: %d\n",FOC_Para_M[0].Motor_State_.controller);
	vTaskDelay(20);
	return 1;
}
/******************************************************************************/
void loopFOC(uint8_t Motor_No)
{
	if( FOC_Para_M[Motor_No].Motor_State_.controller==Type_angle_openloop || FOC_Para_M[Motor_No].Motor_State_.controller==Type_velocity_openloop ) return;
	
	FOC_Para_M[Motor_No].Motor_State_.shaft_angle = shaftAngle(Motor_No);// shaft angle
	FOC_Para_M[Motor_No].Motor_State_.electrical_angle = electricalAngle(Motor_No);// electrical angle - need shaftAngle to be called first
	
	switch(FOC_Para_M[Motor_No].Motor_State_.torque_controller)
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
  setPhaseVoltage(Motor_No,FOC_Para_M[Motor_No].Motor_State_.voltage.q, FOC_Para_M[Motor_No].Motor_State_.voltage.d, FOC_Para_M[Motor_No].Motor_State_.electrical_angle);
}
/******************************************************************************/
void move(uint8_t Motor_No,float new_target)
{
	FOC_Para_M[Motor_No].Motor_State_.shaft_velocity = shaftVelocity(Motor_No);
	
	switch(FOC_Para_M[Motor_No].Motor_State_.controller )
	{
		case Type_torque:
			if(FOC_Para_M[Motor_No].Motor_State_.torque_controller ==Type_voltage)FOC_Para_M[Motor_No].Motor_State_.voltage.q = new_target;  // if voltage torque control
		  else
				FOC_Para_M[Motor_No].Motor_State_.current_sp = new_target; // if current/foc_current torque control
			break;
		case Type_angle:
			// angle set point
      FOC_Para_M[Motor_No].Motor_State_.shaft_angle_sp = new_target;
      // calculate velocity set point
      FOC_Para_M[Motor_No].Motor_State_.shaft_velocity_sp = PID_angle(Motor_No,FOC_Para_M[Motor_No].Motor_State_.shaft_angle_sp - FOC_Para_M[Motor_No].Motor_State_.shaft_angle );
      // calculate the torque command
      FOC_Para_M[Motor_No].Motor_State_.current_sp = PID_velocity(Motor_No,FOC_Para_M[Motor_No].Motor_State_.shaft_velocity_sp - FOC_Para_M[Motor_No].Motor_State_.shaft_velocity); // if voltage torque control
      // if torque controlled through voltage  
      if(FOC_Para_M[Motor_No].Motor_State_.torque_controller == Type_voltage)
			{
				FOC_Para_M[Motor_No].Motor_State_.voltage.q = FOC_Para_M[Motor_No].Motor_State_.current_sp;
        FOC_Para_M[Motor_No].Motor_State_.voltage.d = 0;
      }
			break;
		case Type_velocity:
			// velocity set point
      FOC_Para_M[Motor_No].Motor_State_.shaft_velocity_sp = new_target;
      // calculate the torque command
      FOC_Para_M[Motor_No].Motor_State_.current_sp = PID_velocity(Motor_No,FOC_Para_M[Motor_No].Motor_State_.shaft_velocity_sp - FOC_Para_M[Motor_No].Motor_State_.shaft_velocity); // if current/foc_current torque control
      // if torque controlled through voltage control 
      if(FOC_Para_M[Motor_No].Motor_State_.torque_controller == Type_voltage)
			{
        FOC_Para_M[Motor_No].Motor_State_.voltage.q = FOC_Para_M[Motor_No].Motor_State_.current_sp;  // use voltage if phase-resistance not provided
        FOC_Para_M[Motor_No].Motor_State_.voltage.d = 0;
      }
			break;
		case Type_velocity_openloop:
			// velocity control in open loop
      FOC_Para_M[Motor_No].Motor_State_.shaft_velocity_sp = new_target;
      FOC_Para_M[Motor_No].Motor_State_.voltage.q = velocityOpenloop(Motor_No,FOC_Para_M[Motor_No].Motor_State_.shaft_velocity_sp); // returns the voltage that is set to the motor
      FOC_Para_M[Motor_No].Motor_State_.voltage.d = 0;
			break;
		case Type_angle_openloop:
			// angle control in open loop
      FOC_Para_M[Motor_No].Motor_State_.shaft_angle_sp = new_target;
      FOC_Para_M[Motor_No].Motor_State_.voltage.q = angleOpenloop(Motor_No,FOC_Para_M[Motor_No].Motor_State_.shaft_angle_sp); // returns the voltage that is set to the motor
      FOC_Para_M[Motor_No].Motor_State_.voltage.d = 0;
			break;
	}
}
/******************************************************************************/
// 设置PWM到控制器输出
void setPwm(uint8_t Motor_No,float Ua, float Ub, float Uc) 
{
	  // 限制上限
  Ua = _constrain(Ua, 0.0f, FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply);
  Ub = _constrain(Ub, 0.0f, FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply);
  Uc = _constrain(Uc, 0.0f, FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply, 0.0f , 1.0f );

  //写入PWM到PWM 0 1 2 通道
	Set_TIM_PWM_CCRx(FOC_Para_M[Motor_No].Motor_Properties_.TimHandle,PWM_1,dc_a);
	Set_TIM_PWM_CCRx(FOC_Para_M[Motor_No].Motor_Properties_.TimHandle,PWM_2,dc_b);
	Set_TIM_PWM_CCRx(FOC_Para_M[Motor_No].Motor_Properties_.TimHandle,PWM_3,dc_c);
}
void setPhaseVoltage(uint8_t Motor_No,float Uq, float Ud, float angle_el)
{
#if USE_PARK_
	float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
	//	Sensor_update(FOC_Init_Para.Sensor_AS5600_); //更新传感器数值
  Uq=_constrain(Uq,-((FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply)/2),
									(FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply)/2);
//  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + (FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply)/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + (FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply)/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + (FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Power_Supply)/2;
  setPwm(Motor_No,Ua,Ub,Uc);
#else
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	
	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if(Uout> 0.577)Uout= 0.577;
	if(Uout<-0.577)Uout=-0.577;
	
	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
	Set_TIM_PWM_CCRx(FOC_Para_M[Motor_No].Motor_Properties_.TimHandle,PWM_1,dc_a);
	Set_TIM_PWM_CCRx(FOC_Para_M[Motor_No].Motor_Properties_.TimHandle,PWM_2,dc_b);
	Set_TIM_PWM_CCRx(FOC_Para_M[Motor_No].Motor_Properties_.TimHandle,PWM_3,dc_c);
#endif
}
/******************************************************************************/
float velocityOpenloop(uint8_t Motor_No,float target_velocity)
{
	float Ts,Uq;
	
	Ts = (float)Get_Interval_Timetick(&FOC_Para_M[Motor_No].Open_Loop_Param_.Open_Loop_Timetick)/1e-3f;
  // quick fix for strange cases (micros overflow)
  if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f; 
	
	// calculate the necessary angle to achieve target velocity
  FOC_Para_M[Motor_No].Motor_State_.shaft_angle = _normalizeAngle(FOC_Para_M[Motor_No].Motor_State_.shaft_angle  + target_velocity*Ts); 
	
	Uq = FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Motor_No,Uq,  0, _electricalAngle(FOC_Para_M[Motor_No].Motor_State_.shaft_angle, FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs));
	
	return Uq;
}
/******************************************************************************/
float angleOpenloop(uint8_t Motor_No,float target_angle)
{
	float Ts,Uq;
	
	Ts = (float)Get_Interval_Timetick(&FOC_Para_M[Motor_No].Open_Loop_Param_.Open_Loop_Timetick)/1e-3f;
  // quick fix for strange cases (micros overflow)
  if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f; 
	
	// calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  if(fabs( target_angle - FOC_Para_M[Motor_No].Motor_State_.shaft_angle ) > FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit*Ts)
	{
    FOC_Para_M[Motor_No].Motor_State_.shaft_angle += _sign(target_angle - FOC_Para_M[Motor_No].Motor_State_.shaft_angle) * FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit * Ts;
    //shaft_velocity = velocity_limit;
  }
	else
	{
    FOC_Para_M[Motor_No].Motor_State_.shaft_angle = target_angle;
    //shaft_velocity = 0;
  }
	
	Uq = FOC_Para_M[Motor_No].Motor_Properties_.Voltage_Limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Motor_No,Uq,  0, _electricalAngle(FOC_Para_M[Motor_No].Motor_State_.shaft_angle, FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs));

  return Uq;
}
/******************************************************************************/



