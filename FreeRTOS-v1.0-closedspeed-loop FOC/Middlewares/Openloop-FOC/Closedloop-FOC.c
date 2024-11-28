#include "main.h"
#include "cmsis_os.h"

float voltage_power_limit = 12.4;
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
float zero_electric_angle=0;
uint8_t FOC_Iint_Flag = 0;
//int PP=7,DIR=1;

FOC_Init_Para_t FOC_Init_Parameter[FOC_Init_Para_Num];

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


// 设置PWM到控制器输出
void setPwm(FOC_Init_Para_t * FOC_Init_Para,float Ua, float Ub, float Uc) 
{
	  // 限制上限
  Ua = _constrain(Ua, 0.0f, FOC_Init_Para->power_supply_t);
  Ub = _constrain(Ub, 0.0f, FOC_Init_Para->power_supply_t);
  Uc = _constrain(Uc, 0.0f, FOC_Init_Para->power_supply_t);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / FOC_Init_Para->power_supply_t, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / FOC_Init_Para->power_supply_t, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / FOC_Init_Para->power_supply_t, 0.0f , 1.0f );

  //写入PWM到PWM 0 1 2 通道
	Set_TIM3_PWM_CCRx(PWM_1,dc_a);
	Set_TIM3_PWM_CCRx(PWM_2,dc_b);
	Set_TIM3_PWM_CCRx(PWM_3,dc_c);
}

void setTorque(FOC_Init_Para_t * FOC_Init_Para,float Uq,float angle_el) 
{
	Sensor_update(FOC_Init_Para->Sensor_AS5600_); //更新传感器数值
  Uq=_constrain(Uq,-((FOC_Init_Para->power_supply_t)/2),(FOC_Init_Para->power_supply_t)/2);
//  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // 帕克逆变换
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  Ua = Ualpha + (FOC_Init_Para->power_supply_t)/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + (FOC_Init_Para->power_supply_t)/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + (FOC_Init_Para->power_supply_t)/2;
  setPwm(FOC_Init_Para,Ua,Ub,Uc);
}
//=================PID 设置函数=================
//速度PID
void DFOC_M0_SET_VEL_PID(FOC_Init_Para_t* FOC_Init_Para_,float P,float I,float D,float ramp)   //M0角度环PID设置
{
  FOC_Init_Para_->PID_Vel_Loop_->P=P;
  FOC_Init_Para_->PID_Vel_Loop_->I=I;
  FOC_Init_Para_->PID_Vel_Loop_->D=D;
  FOC_Init_Para_->PID_Vel_Loop_->output_ramp=ramp;
}

//角度PID
void DFOC_M0_SET_ANGLE_PID(FOC_Init_Para_t* FOC_Init_Para_,float P,float I,float D,float ramp)   //M0角度环PID设置
{
  FOC_Init_Para_->PID_Angle_Loop_->P=P;
  FOC_Init_Para_->PID_Angle_Loop_->I=I;
  FOC_Init_Para_->PID_Angle_Loop_->D=D;
  FOC_Init_Para_->PID_Angle_Loop_->output_ramp=ramp;
}

//M0速度PID接口
float DFOC_M0_VEL_PID(FOC_Init_Para_t* FOC_Init_Para_,float error)   //M0速度环
{
		LCD_Show_Parameter.PID_Error_Show = error;
   return PIDController_Operator(FOC_Init_Para_->PID_Vel_Loop_,error);
}
//M0角度PID接口
float DFOC_M0_ANGLE_PID(FOC_Init_Para_t* FOC_Init_Para_,float error)
{
	LCD_Show_Parameter.PID_Error_Show = error;
  return PIDController_Operator(FOC_Init_Para_->PID_Angle_Loop_,error);
}

float _electricalAngle(FOC_Init_Para_t* FOC_Init_Para_){
  return  _normalizeAngle((float)(FOC_Init_Para_->Sensor_DIR *  FOC_Init_Para_->Motor_PP) 
							* getMechanicalAngle(FOC_Init_Para_->Sensor_AS5600_)-zero_electric_angle);
}


float DFOC_M0_Angle(FOC_Init_Para_t * FOC_Init_Para)
{
  return FOC_Init_Para->Sensor_DIR*getAngle(FOC_Init_Para->Sensor_AS5600_);
}

//无滤波
//float DFOC_M0_Velocity(FOC_Init_Para_t * FOC_Init_Para)
//{
//  return FOC_Init_Para->Sensor_DIR*getVelocity(FOC_Init_Para->Sensor_AS5600_);
//}

//有滤波
float DFOC_M0_Velocity(FOC_Init_Para_t * FOC_Init_Para)
{
  //获取速度数据并滤波
  float vel_M0_ori=getVelocity(FOC_Init_Para->Sensor_AS5600_);
  float vel_M0_flit=LowPassFilter_Operator(FOC_Init_Para->Vel_Lowpass_Filter_,
																				FOC_Init_Para->Sensor_DIR*vel_M0_ori);
	LCD_Show_Parameter.xCurrent_Vel_Show = vel_M0_flit;
  return vel_M0_flit;   //考虑方向
}

//================简易接口函数================
void DFOC_M0_set_Velocity_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target)//角度闭环
{
	Target = FOC_Init_Para->Sensor_DIR*Target;
 setTorque(FOC_Init_Para,DFOC_M0_VEL_PID(FOC_Init_Para,DFOC_M0_ANGLE_PID(FOC_Init_Para,(Target-DFOC_M0_Angle(FOC_Init_Para))*180/PI)),_electricalAngle(FOC_Init_Para));   
}

void DFOC_M0_setVelocity(FOC_Init_Para_t * FOC_Init_Para,float Target)//速度闭环
{
	Target = FOC_Init_Para->Sensor_DIR*Target;
  setTorque(FOC_Init_Para,DFOC_M0_VEL_PID(FOC_Init_Para,(Target-DFOC_M0_Velocity(FOC_Init_Para))*180/PI),_electricalAngle(FOC_Init_Para));   
}


void DFOC_M0_set_Force_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target)   //力位
{
	Target = FOC_Init_Para->Sensor_DIR*Target;
  setTorque(FOC_Init_Para,DFOC_M0_ANGLE_PID(FOC_Init_Para,(Target-DFOC_M0_Angle(FOC_Init_Para))*180/PI),_electricalAngle(FOC_Init_Para));
}

void DFOC_M0_setTorque(FOC_Init_Para_t * FOC_Init_Para,float Target)
{
	Target = FOC_Init_Para->Sensor_DIR*Target;
  setTorque(FOC_Init_Para,Target,_electricalAngle(FOC_Init_Para));
}


void DFOC_alignSensor(FOC_Init_Para_t * FOC_Init_Para)
{ 

  setTorque(FOC_Init_Para,3, _3PI_2);
  vTaskDelay(1500);
  zero_electric_angle=_electricalAngle(FOC_Init_Para);
  setTorque(FOC_Init_Para,0, _3PI_2);
  LCD_Show_Parameter.zero_electric_angle_Show =zero_electric_angle;
}

void FOC_Iint(FOC_Init_Para_t * FOC_Init_Para,uint8_t FOC_No_,
							float power_supply_,int _PP,int _DIR,
							I2C_HandleTypeDef* _hi2c,float time_constant,
							PID_Setting_Coefficient_t* PID_Vel_Setting_,
							PID_Setting_Coefficient_t* PID_Angle_Setting_)
{
	FOC_Init_Para->FOC_No_Num = FOC_No_;
	if(fabsf(power_supply_)>=voltage_power_limit)
	{
		power_supply_=voltage_power_limit;
	}
	FOC_Init_Para->power_supply_t = power_supply_;
	FOC_Init_Para->Motor_PP = _PP;
	FOC_Init_Para->Sensor_DIR = _DIR;
	FOC_Init_Para->Sensor_AS5600_ = &Sensor_AS5600_Parameter[FOC_Init_Para->FOC_No_Num];
	FOC_Init_Para->Angle_Lowpass_Filter_ = &Lowpass_Angle_Filter_Para[FOC_Init_Para->FOC_No_Num];
	FOC_Init_Para->Vel_Lowpass_Filter_ = &Lowpass_Vel_Filter_Para[FOC_Init_Para->FOC_No_Num];
	FOC_Init_Para->PID_Vel_Loop_ = &PID_Vel_Loop_Parameter[FOC_Init_Para->FOC_No_Num];
	FOC_Init_Para->PID_Angle_Loop_ = &PID_Angle_Loop_Parameter[FOC_Init_Para->FOC_No_Num];
	
	Sensor_init(FOC_Init_Para->Sensor_AS5600_,FOC_No_,_hi2c);
	
	LowPassFilter_Init(FOC_Init_Para->Angle_Lowpass_Filter_,time_constant);
	LowPassFilter_Init(FOC_Init_Para->Vel_Lowpass_Filter_,time_constant);
	
	PID_Vel_Setting_->limit = FOC_Init_Para->power_supply_t/2;
	PIDController_Init(FOC_Init_Para->PID_Vel_Loop_,PID_Vel_Setting_->P,
											PID_Vel_Setting_->I,PID_Vel_Setting_->D,
											PID_Vel_Setting_->ramp,PID_Vel_Setting_->limit);
	
	PID_Angle_Setting_->limit = FOC_Init_Para->power_supply_t/2;
	PIDController_Init(FOC_Init_Para->PID_Angle_Loop_,PID_Angle_Setting_->P,
											PID_Angle_Setting_->I,PID_Angle_Setting_->D,
											PID_Angle_Setting_->ramp,PID_Angle_Setting_->limit);
											
	First_Time_Laod(&FOC_Init_Para->FOC_Task_Timecount);
											
	DFOC_alignSensor(FOC_Init_Para);
	FOC_Iint_Flag = 1;
}
