#ifndef BLDCMotor_H
#define BLDCMotor_H


#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stm32f4xx_hal.h"
#include "foc_utils.h"
#include "FOC_pid.h"
#include "lowpass_filter.h"
#include "MagneticSensor.h"
#include "FOCMotor.h"
#include "Timecount.h" 

/******************************************************************************/
/**
 *  Direction structure
 */

typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;


typedef struct {
		uint8_t FOC_No_Num;
		TIM_HandleTypeDef* TimHandle;
		float Voltage_Power_Supply;
		float Voltage_Limit;
		float Voltage_Sensor_Align;
		int Motor_Pole_Pairs;
		int Sensor_DIR;
} Motor_Properties_t;

typedef struct {
		Interval_Timetick_t Open_Loop_Timetick;
		float Openloop_Velocity_Limit;
} Open_Loop_Param_t;




typedef struct {
	Motor_Properties_t	 	Motor_Properties_;
	Motor_State_t 				Motor_State_;
	MagneticSensor_t 			MagneticSensor_;
	Open_Loop_Param_t 		Open_Loop_Param_;
	PID_Vel_t							PID_Vel_;
	PID_Angle_t						PID_Angle_;
} FOC_Para_All_t;



extern FOC_Para_All_t FOC_Para_M[2];

/******************************************************************************/
void Motor_init(uint8_t Motor_No);
void Motor_initFOC(uint8_t Motor_No);
void loopFOC(uint8_t Motor_No);
void move(uint8_t Motor_No,float new_target);
void setPhaseVoltage(uint8_t Motor_No,float Uq, float Ud, float angle_el);
/******************************************************************************/


#ifdef __cplusplus
}
#endif
#endif
