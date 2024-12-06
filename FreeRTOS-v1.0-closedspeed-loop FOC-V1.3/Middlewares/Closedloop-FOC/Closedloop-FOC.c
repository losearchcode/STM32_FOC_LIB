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
//	Sensor_update(FOC_Init_Para->Sensor_AS5600_); //���´�������ֵ
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
	
	
//	float Uref;
//	uint32_t sector;
//	float T0,T1,T2;
//	float Ta,Tb,Tc;
//	float U_alpha,U_beta;
//	
//	angle_el =_normalizeAngle(angle_el);                    //��׼����:0_2PI
// 
//	U_alpha=-Uq*sin(angle_el);            //����park��任
//	U_beta=Uq*cos(angle_el);
//	
//	Uref=_sqrt(U_alpha*U_alpha + U_beta*U_beta) / (FOC_Init_Para->power_supply_t);
//	
//	if(Uref> 0.577)Uref= 0.577;  //����������Բ��SVPWM���ʧ����ת��ѹʸ����sqrt(3)/3
//	if(Uref<-0.577)Uref=-0.577; 
//	
//	if(Uq>0)
//	  angle_el =_normalizeAngle(angle_el+_PI_2);            //+90�ο���ѹʸ��λ��
//	else
//		angle_el =_normalizeAngle(angle_el-_PI_2);
//	sector = (angle_el / _PI_3) + 1;                			//���ݽǶ��жϲο���ѹ��������
// 
//	T1 = _SQRT3*sin(sector*_PI_3 - angle_el) * Uref;           //�������ڵ�ѹʸ������ʱ��
//	T2 = _SQRT3*sin(angle_el - (sector-1.0)*_PI_3) * Uref;
//	T0 = 1 - T1 - T2;                                          //��ʸ������ʱ��
//	
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
//	Set_TIM3_PWM_CCRx(PWM_1,Ta);
//	Set_TIM3_PWM_CCRx(PWM_2,Tb);
//	Set_TIM3_PWM_CCRx(PWM_3,Tc);
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
 setTorque(FOC_Init_Para,DFOC_M0_VEL_PID(FOC_Init_Para,DFOC_M0_ANGLE_PID(FOC_Init_Para,(Target-DFOC_M0_Angle(FOC_Init_Para))*180/PI)),_electricalAngle(FOC_Init_Para));   
}

void DFOC_M0_setVelocity(FOC_Init_Para_t * FOC_Init_Para,float Target)//�ٶȱջ�
{
  setTorque(FOC_Init_Para,DFOC_M0_VEL_PID(FOC_Init_Para,(Target-DFOC_M0_Velocity(FOC_Init_Para))*180/PI),_electricalAngle(FOC_Init_Para));   
}


void DFOC_M0_set_Force_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target)   //��λ
{
  setTorque(FOC_Init_Para,DFOC_M0_ANGLE_PID(FOC_Init_Para,(Target-DFOC_M0_Angle(FOC_Init_Para))*180/PI),_electricalAngle(FOC_Init_Para));
}

void DFOC_M0_setTorque(FOC_Init_Para_t * FOC_Init_Para,float Target)
{
  setTorque(FOC_Init_Para,Target,_electricalAngle(FOC_Init_Para));
}


void DFOC_alignSensor(FOC_Init_Para_t * FOC_Init_Para)
{ 

  long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	float voltage_sensor_align=2;
	
	printf("MOT: Align sensor.\r\n");
	
	// find natural direction
	// move one electrical revolution forward
	for(i=0; i<=500; i++)
	{
		angle = _3PI_2 + _2PI * i / 500.0;
		setTorque(FOC_Init_Para,voltage_sensor_align,angle);
		vTaskDelay(2);
	}
	mid_angle=getAngle(FOC_Init_Para->Sensor_AS5600_);
	
	for(i=500; i>=0; i--) 
	{
		angle = _3PI_2 + _2PI * i / 500.0 ;
		setTorque(FOC_Init_Para,voltage_sensor_align,  angle);
		vTaskDelay(2);
	}
	end_angle=getAngle(FOC_Init_Para->Sensor_AS5600_);
	setTorque(FOC_Init_Para,0, 0);
	vTaskDelay(200);
	
	printf("mid_angle=%.4f\r\n",mid_angle);
	printf("end_angle=%.4f\r\n",end_angle);
	
	moved =  fabs(mid_angle - end_angle);
	if((mid_angle == end_angle)||(moved < 0.02))  //��Ȼ��߼���û�ж�
	{
		printf("MOT: Failed to notice movement loop222.\r\n");
		FOC_EN(OFF);   //�����ⲻ�������ر�����
	}
	else if(mid_angle < end_angle)
	{
		printf("MOT: sensor_direction==CCW\r\n");
		FOC_Init_Para->Sensor_DIR=-1;
	}
	else
	{
		printf("MOT: sensor_direction==CW\r\n");
		FOC_Init_Para->Sensor_DIR=1;
	}
	
	
	printf("MOT: PP check: ");    //����Pole_Pairs
	if( fabs(moved*FOC_Init_Para->Motor_PP - _2PI) > 0.5 )  // 0.5 is arbitrary number it can be lower or higher!
	{
		printf("fail - estimated pp:");
		FOC_Init_Para->Motor_PP=_2PI/moved+0.5;     //������ת���Σ���������
		printf("%d\r\n",FOC_Init_Para->Motor_PP);
  }
	else
		printf("OK!\r\n");
	
	
	setTorque(FOC_Init_Para,voltage_sensor_align,  _3PI_2);  //�������ƫ�ƽǶ�
	vTaskDelay(700);
	zero_electric_angle = _normalizeAngle(_electricalAngle(FOC_Init_Para));
	vTaskDelay(20);
	printf("MOT: Zero elec. angle:");
	printf("%.4f\r\n",zero_electric_angle);
	
	setTorque(FOC_Init_Para, 0, 0);
	vTaskDelay(200);
	
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
