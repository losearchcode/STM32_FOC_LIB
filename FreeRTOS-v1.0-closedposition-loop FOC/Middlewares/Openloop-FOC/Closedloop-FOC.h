#ifndef __OPEN_LOOP_FOC_H
#define __OPEN_LOOP_FOC_H

#include "main.h"

#define PI 3.14159265358979323846f
#define _3PI_2 4.71238898038f

typedef struct {
		float power_supply_t;
		int Motor_PP;
		int Sensor_DIR;
		float xCurrent_Sensor_Angle;
		float xCurrent_electricalAngle;
} FOC_Init_Para_t;

extern FOC_Init_Para_t FOC_Init_Parameter;
extern uint8_t FOC_Iint_Flag;

//��ʼ��������������
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//�궨��ʵ�ֵ�һ��Լ������,��������һ��ֵ�ķ�Χ��
//������˵���ú궨�������Ϊ _constrain�������������� amt��low �� high���ֱ��ʾҪ���Ƶ�ֵ����Сֵ�����ֵ���ú궨���ʵ��ʹ������Ԫ����������� amt �Ƿ�С�� low ����� high���������е�������Сֵ�����߷���ԭֵ��
//���仰˵����� amt С�� low���򷵻� low����� amt ���� high���򷵻� high�����򷵻� amt��������_constrain(amt, low, high) �ͻὫ amt Լ���� [low, high] �ķ�Χ�ڡ�

void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq,float angle_el);
float _normalizeAngle(float angle);
void DFOC_Vbus(float power_supply);
void DFOC_alignSensor(int _PP,int _DIR);
float _electricalAngle(void);
float _electricalAngle_AS5600(float val);
float DFOC_M0_Angle(void);
void FOC_Iint(void);

#endif
