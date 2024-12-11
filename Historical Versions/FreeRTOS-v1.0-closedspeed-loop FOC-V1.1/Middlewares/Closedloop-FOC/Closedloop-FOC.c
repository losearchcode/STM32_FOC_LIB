#include "main.h"
#include "cmsis_os.h"

float voltage_power_limit = 12.4;
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
float zero_electric_angle=0;
uint8_t FOC_Iint_Flag = 0;
//int PP=7,DIR=1;

FOC_Init_Para_t FOC_Init_Parameter[FOC_Init_Para_Num];

// ��һ���Ƕȵ� [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //ȡ������������ڹ�һ�����г�����ֵ�������֪
  return a >= 0 ? a : (a + 2*PI);  
  //��Ŀ���������ʽ��condition ? expr1 : expr2 
  //���У�condition ��Ҫ��ֵ���������ʽ����������������򷵻� expr1 ��ֵ�����򷵻� expr2 ��ֵ�����Խ���Ŀ�������Ϊ if-else ���ļ���ʽ��
  //fmod �����������ķ����������ͬ����ˣ��� angle ��ֵΪ����ʱ�������ķ��Ž��� _2PI �ķ����෴��Ҳ����˵����� angle ��ֵС�� 0 �� _2PI ��ֵΪ�������� fmod(angle, _2PI) ��������Ϊ������
  //���磬�� angle ��ֵΪ -PI/2��_2PI ��ֵΪ 2PI ʱ��fmod(angle, _2PI) ������һ������������������£�����ͨ������������������ _2PI �����Ƕȹ�һ���� [0, 2PI] �ķ�Χ�ڣ���ȷ���Ƕȵ�ֵʼ��Ϊ������
}


// ����PWM�����������
void setPwm(FOC_Init_Para_t * FOC_Init_Para,float Ua, float Ub, float Uc) 
{
	  // ��������
  Ua = _constrain(Ua, 0.0f, FOC_Init_Para->power_supply_t);
  Ub = _constrain(Ub, 0.0f, FOC_Init_Para->power_supply_t);
  Uc = _constrain(Uc, 0.0f, FOC_Init_Para->power_supply_t);
  // ����ռ�ձ�
  // ����ռ�ձȴ�0��1
  float dc_a = _constrain(Ua / FOC_Init_Para->power_supply_t, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / FOC_Init_Para->power_supply_t, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / FOC_Init_Para->power_supply_t, 0.0f , 1.0f );

  //д��PWM��PWM 0 1 2 ͨ��
	Set_TIM3_PWM_CCRx(PWM_1,dc_a);
	Set_TIM3_PWM_CCRx(PWM_2,dc_b);
	Set_TIM3_PWM_CCRx(PWM_3,dc_c);
}

void setTorque(FOC_Init_Para_t * FOC_Init_Para,float Uq,float angle_el) 
{
	Sensor_update(FOC_Init_Para->Sensor_AS5600_); //���´�������ֵ
  Uq=_constrain(Uq,-((FOC_Init_Para->power_supply_t)/2),(FOC_Init_Para->power_supply_t)/2);
//  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // ������任
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // ��������任
  Ua = Ualpha + (FOC_Init_Para->power_supply_t)/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + (FOC_Init_Para->power_supply_t)/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + (FOC_Init_Para->power_supply_t)/2;
  setPwm(FOC_Init_Para,Ua,Ub,Uc);
}
//=================PID ���ú���=================
//�ٶ�PID
void DFOC_M0_SET_VEL_PID(FOC_Init_Para_t* FOC_Init_Para_,float P,float I,float D,float ramp)   //M0�ǶȻ�PID����
{
  FOC_Init_Para_->PID_Vel_Loop_->P=P;
  FOC_Init_Para_->PID_Vel_Loop_->I=I;
  FOC_Init_Para_->PID_Vel_Loop_->D=D;
  FOC_Init_Para_->PID_Vel_Loop_->output_ramp=ramp;
}

//�Ƕ�PID
void DFOC_M0_SET_ANGLE_PID(FOC_Init_Para_t* FOC_Init_Para_,float P,float I,float D,float ramp)   //M0�ǶȻ�PID����
{
  FOC_Init_Para_->PID_Angle_Loop_->P=P;
  FOC_Init_Para_->PID_Angle_Loop_->I=I;
  FOC_Init_Para_->PID_Angle_Loop_->D=D;
  FOC_Init_Para_->PID_Angle_Loop_->output_ramp=ramp;
}

//M0�ٶ�PID�ӿ�
float DFOC_M0_VEL_PID(FOC_Init_Para_t* FOC_Init_Para_,float error)   //M0�ٶȻ�
{
		LCD_Show_Parameter.PID_Error_Show = error;
   return PIDController_Operator(FOC_Init_Para_->PID_Vel_Loop_,error);
}
//M0�Ƕ�PID�ӿ�
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

//���˲�
//float DFOC_M0_Velocity(FOC_Init_Para_t * FOC_Init_Para)
//{
//  return FOC_Init_Para->Sensor_DIR*getVelocity(FOC_Init_Para->Sensor_AS5600_);
//}

//���˲�
float DFOC_M0_Velocity(FOC_Init_Para_t * FOC_Init_Para)
{
  //��ȡ�ٶ����ݲ��˲�
  float vel_M0_ori=getVelocity(FOC_Init_Para->Sensor_AS5600_);
  float vel_M0_flit=LowPassFilter_Operator(FOC_Init_Para->Vel_Lowpass_Filter_,
																				FOC_Init_Para->Sensor_DIR*vel_M0_ori);
	LCD_Show_Parameter.xCurrent_Vel_Show = vel_M0_flit;
  return vel_M0_flit;   //���Ƿ���
}

//================���׽ӿں���================
void DFOC_M0_set_Velocity_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target)//�Ƕȱջ�
{
	Target = FOC_Init_Para->Sensor_DIR*Target;
 setTorque(FOC_Init_Para,DFOC_M0_VEL_PID(FOC_Init_Para,DFOC_M0_ANGLE_PID(FOC_Init_Para,(Target-DFOC_M0_Angle(FOC_Init_Para))*180/PI)),_electricalAngle(FOC_Init_Para));   
}

void DFOC_M0_setVelocity(FOC_Init_Para_t * FOC_Init_Para,float Target)//�ٶȱջ�
{
	Target = FOC_Init_Para->Sensor_DIR*Target;
  setTorque(FOC_Init_Para,DFOC_M0_VEL_PID(FOC_Init_Para,(Target-DFOC_M0_Velocity(FOC_Init_Para))*180/PI),_electricalAngle(FOC_Init_Para));   
}


void DFOC_M0_set_Force_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target)   //��λ
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
