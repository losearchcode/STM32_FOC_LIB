#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"

#define _2PI 6.28318530718f

#define AS5600_NUM FOC_Motor_Num

typedef struct {
    int Mot_Num;
    I2C_HandleTypeDef* hi2c;  // I2C通信接口
    float Angle_prev;  // 上一次的角度
		Interval_Timetick_t AS5600_Angle_Timecount;
    uint32_t Angle_Interval_Ts;  // 上一次角度的时间戳
    float Vel_angle_prev;  // 用于计算速度的上一次角度
		Interval_Timetick_t AS5600_Vel_Timecount;
    uint32_t Vel_Interval_ts;  // 用于计算速度的上一次时间戳
    int full_rotations;  // 完整圈数
    int Vel_full_rotations;  // 用于计算速度的完整圈数
		float Current_Vel;
} Sensor_AS5600;


extern Sensor_AS5600 Sensor_AS5600_Parameter[AS5600_NUM];


double getSensorAngle(Sensor_AS5600* sensor);
void Sensor_init(Sensor_AS5600* sensor,int _Mot_Num,I2C_HandleTypeDef* _hi2c);
void Sensor_update(Sensor_AS5600* sensor);
float getMechanicalAngle(Sensor_AS5600* sensor);
float getAngle(Sensor_AS5600* sensor);
float getVelocity(Sensor_AS5600* sensor);

#endif /* __AS5600_H */
