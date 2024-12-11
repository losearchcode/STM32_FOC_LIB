#include "main.h"
#include "cmsis_os.h"

float voltage_power_supply;
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0;
float zero_electric_angle=0;
uint8_t FOC_Iint_Flag = 0;
int PP=7,DIR=1;

FOC_Init_Para_t FOC_Init_Parameter;

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
void setPwm(float Ua, float Ub, float Uc) 
{
  // ����ռ�ձ�
  // ����ռ�ձȴ�0��1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //д��PWM��PWM 0 1 2 ͨ��
	Set_TIM3_PWM_CCRx(PWM_1,dc_a);
	Set_TIM3_PWM_CCRx(PWM_2,dc_b);
	Set_TIM3_PWM_CCRx(PWM_3,dc_c);
}

void setTorque(float Uq,float angle_el) 
{
  Uq=_constrain(Uq,-voltage_power_supply/2,voltage_power_supply/2);
//  float Ud=0;
  angle_el = _normalizeAngle(angle_el);
  // ������任
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // ��������任
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;
}


float _electricalAngle(void)
{
  return  _normalizeAngle((float)(DIR *  PP) * getAngle_Without_track()-zero_electric_angle);
}

float _electricalAngle_AS5600(float val)
{
  return  _normalizeAngle((float)(DIR *  PP) * val-zero_electric_angle);
}


void DFOC_alignSensor(int _PP,int _DIR)
{ 
  PP=_PP;
  DIR=_DIR;
  setTorque(3, _3PI_2);
  vTaskDelay(3000);
  zero_electric_angle=_electricalAngle();
  setTorque(0, _3PI_2);
  LCD_Show_Parameter.zero_electric_angle_Show =zero_electric_angle;
}

float DFOC_M0_Angle(void)
{
  return getAngle();
}

void FOC_Iint(void)
{
	DFOC_Vbus(FOC_Init_Parameter.power_supply_t);
	DFOC_alignSensor(FOC_Init_Parameter.Motor_PP,FOC_Init_Parameter.Sensor_DIR);
	FOC_Iint_Flag = 1;
}

