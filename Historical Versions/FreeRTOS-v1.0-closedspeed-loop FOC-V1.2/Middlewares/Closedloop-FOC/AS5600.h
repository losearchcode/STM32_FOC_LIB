#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"

#define _2PI 6.28318530718f

#define AS5600_NUM FOC_Motor_Num

typedef struct {
    int Mot_Num;
    I2C_HandleTypeDef* hi2c;  // I2Cͨ�Žӿ�
    float Angle_prev;  // ��һ�εĽǶ�
		Interval_Timetick_t AS5600_Angle_Timecount;
    uint32_t Angle_Interval_Ts;  // ��һ�νǶȵ�ʱ���
    float Vel_angle_prev;  // ���ڼ����ٶȵ���һ�νǶ�
		Interval_Timetick_t AS5600_Vel_Timecount;
    uint32_t Vel_Interval_ts;  // ���ڼ����ٶȵ���һ��ʱ���
    int full_rotations;  // ����Ȧ��
    int Vel_full_rotations;  // ���ڼ����ٶȵ�����Ȧ��
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
