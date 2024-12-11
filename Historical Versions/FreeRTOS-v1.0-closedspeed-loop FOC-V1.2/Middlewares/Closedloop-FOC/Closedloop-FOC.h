#ifndef __OPEN_LOOP_FOC_H
#define __OPEN_LOOP_FOC_H


#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "main.h"



#define PI 3.14159265358979323846f
#define _3PI_2 4.71238898038f

#define _2_SQRT3 1.15470053838
#define _SQRT3 1.73205080757
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _PI_6 0.52359877559

#define FOC_Init_Para_Num FOC_Motor_Num

typedef struct {
		uint8_t FOC_No_Num;
		float power_supply_t;
		int Motor_PP;
		int Sensor_DIR;
		Sensor_AS5600 * Sensor_AS5600_;
		Lowpass_Filter_t * Vel_Lowpass_Filter_;
		Lowpass_Filter_t * Angle_Lowpass_Filter_;
		PID_Controller_t * PID_Vel_Loop_;
		PID_Controller_t * PID_Angle_Loop_;
		Interval_Timetick_t FOC_Task_Timecount;
} FOC_Init_Para_t;

extern FOC_Init_Para_t FOC_Init_Parameter[FOC_Init_Para_Num];
extern uint8_t FOC_Iint_Flag;

//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。

void setPwm(FOC_Init_Para_t * FOC_Init_Para,float Ua, float Ub, float Uc);
void setTorque(FOC_Init_Para_t * FOC_Init_Para,float Uq,float angle_el); 
float _normalizeAngle(float angle);

float DFOC_M0_ANGLE_PID(FOC_Init_Para_t* FOC_Init_Para_,float error);
float DFOC_M0_VEL_PID(FOC_Init_Para_t* FOC_Init_Para_,float error);
void DFOC_M0_SET_ANGLE_PID(FOC_Init_Para_t* FOC_Init_Para_,float P,float I,float D,float ramp);   //M0角度环PID设置
void DFOC_M0_SET_VEL_PID(FOC_Init_Para_t* FOC_Init_Para_,float P,float I,float D,float ramp);   //M0角度环PID设置

float _electricalAngle(FOC_Init_Para_t* FOC_Init_Para_);
float DFOC_M0_Velocity(FOC_Init_Para_t * FOC_Init_Para);
float DFOC_M0_Angle(FOC_Init_Para_t * FOC_Init_Para);

//================简易接口函数================
void DFOC_M0_set_Velocity_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target);
//角度闭环
void DFOC_M0_setVelocity(FOC_Init_Para_t * FOC_Init_Para,float Target);
//速度闭环
void DFOC_M0_set_Force_Angle(FOC_Init_Para_t * FOC_Init_Para,float Target);
//力位
void DFOC_M0_setTorque(FOC_Init_Para_t * FOC_Init_Para,float Target);
void DFOC_alignSensor(FOC_Init_Para_t * FOC_Init_Para);



void FOC_Iint(FOC_Init_Para_t * FOC_Init_Para,uint8_t FOC_No_,
							float power_supply_,int _PP,int _DIR,
							I2C_HandleTypeDef* _hi2c,float time_constant,
							PID_Setting_Coefficient_t* PID_Vel_Setting_,
							PID_Setting_Coefficient_t* PID_Angle_Setting_);

#ifdef __cplusplus
}
#endif

#endif
