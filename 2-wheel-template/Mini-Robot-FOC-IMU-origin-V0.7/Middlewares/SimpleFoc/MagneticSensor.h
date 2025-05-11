#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H
#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stm32f4xx_hal.h"
#include "Timecount.h" 

typedef struct {
    I2C_HandleTypeDef* hi2c;  // I2C通信接口
    long  Cpr;
		float Full_Rotation_Offset;
    long  Angle_Data_Prev;
		float Angle_Prev;
		Interval_Timetick_t AS5600_Vel_Timecount;
} MagneticSensor_t;


/******************************************************************************/
void MagneticSensor_Init(uint8_t Motor_No ,I2C_HandleTypeDef* hi2c);
float getAngle(uint8_t Motor_No);
float getVelocity(uint8_t Motor_No);
/******************************************************************************/
#ifdef __cplusplus
}
#endif

#endif
