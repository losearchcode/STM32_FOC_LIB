#ifndef __OPEN_LOOP_FOC_H
#define __OPEN_LOOP_FOC_H

#include "main.h"

#define PI 3.14159265358979323846

//��ʼ��������������
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//�궨��ʵ�ֵ�һ��Լ������,��������һ��ֵ�ķ�Χ��
//������˵���ú궨�������Ϊ _constrain�������������� amt��low �� high���ֱ��ʾҪ���Ƶ�ֵ����Сֵ�����ֵ���ú궨���ʵ��ʹ������Ԫ����������� amt �Ƿ�С�� low ����� high���������е�������Сֵ�����߷���ԭֵ��
//���仰˵����� amt С�� low���򷵻� low����� amt ���� high���򷵻� high�����򷵻� amt��������_constrain(amt, low, high) �ͻὫ amt Լ���� [low, high] �ķ�Χ�ڡ�

float velocityOpenloop(float target_velocity,uint32_t xElapsedTime);

#endif
