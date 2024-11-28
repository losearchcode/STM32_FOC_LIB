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

//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。

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
