#include "main.h"
#include "math.h"


float voltage_power_supply=12.4;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
uint8_t  BLDC_Pole_Num = 7;

// ��Ƕ����
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

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
void setPwm(float Ua, float Ub, float Uc) {

  // ����ռ�ձ�
  // ����ռ�ձȴ�0��1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //д��PWM��PWM 0 1 2 ͨ��
	Set_TIM3_PWM_CCRx(PWM_1,dc_a);
	Set_TIM3_PWM_CCRx(PWM_2,dc_b);
	Set_TIM3_PWM_CCRx(PWM_3,dc_c);
}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
  // ������任
  Ualpha =  -Uq*sin(angle_el); 
  Ubeta =   Uq*cos(angle_el); 

  // ��������任
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

//�����ٶȺ���
float velocityOpenloop(float target_velocity,uint32_t xElapsedTime){

  //���㵱ǰÿ��Loop������ʱ����
  float Ts = xElapsedTime * 1e-3f;

  //���� micros() �������ص�ʱ������ڴ�Լ 70 ����֮�����¿�ʼ����������70�������䵽0ʱ��TS������쳣�������Ҫ�������������ʱ����С�ڵ��������� 0.5 �룬��������Ϊһ����С��Ĭ��ֵ���� 1e-3f
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  

  // ͨ������ʱ������Ŀ���ٶ���������Ҫת���Ļ�е�Ƕȣ��洢�� shaft_angle �����С��ڴ�֮ǰ������Ҫ����ǶȽ��й�һ������ȷ����ֵ�� 0 �� 2�� ֮�䡣
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  //��Ŀ���ٶ�Ϊ 10 rad/s Ϊ�������ʱ������ 1 �룬����ÿ��ѭ������Ҫ���� 10 * 1 = 10 ���ȵĽǶȱ仯��������ʹ���ת����Ŀ���ٶȡ�
  //���ʱ������ 0.1 �룬��ô��ÿ��ѭ������Ҫ���ӵĽǶȱ仯������ 10 * 0.1 = 1 ���ȣ�����ʵ����ͬ��Ŀ���ٶȡ���ˣ�������ת���Ƕ�ȡ����Ŀ���ٶȺ�ʱ�����ĳ˻���

  // ʹ����ǰ���õ�voltage_power_supply��1/3��ΪUqֵ�����ֵ��ֱ��Ӱ���������
  // ���ֻ������ΪUq = voltage_power_supply/2������ua,ub,uc�ᳬ�������ѹ�޷�
  float Uq = voltage_power_supply/3;
  
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, BLDC_Pole_Num));
  

  return Uq;
}
