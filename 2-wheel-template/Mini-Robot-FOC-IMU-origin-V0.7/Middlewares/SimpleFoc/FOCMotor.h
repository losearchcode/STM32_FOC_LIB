#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stm32f4xx_hal.h"
	
typedef enum
{
	Type_torque,//!< Torque control
	Type_velocity,//!< Velocity motion control
	Type_angle,//!< Position/angle motion control
	Type_velocity_openloop,
	Type_angle_openloop
} MotionControlType;

/**
 *  Motiron control type
 */
typedef enum
{
	Type_voltage, //!< Torque control using voltage
	Type_dc_current, //!< Torque control using DC current (one current magnitude)
	Type_foc_current //!< torque control using dq currents
} TorqueControlType;

// dq current structure 
typedef struct 
{
	float d;
	float q;
} DQCurrent_s;
// phase current structure 
typedef struct 
{
	float a;
	float b;
	float c;
} PhaseCurrent_s;
// dq voltage structs
typedef struct 
{
	float d;
	float q;
} DQVoltage_s;

typedef struct 
{
	float target;
	float shaft_angle;//!< current motor angle
	float electrical_angle;
	float shaft_velocity;
	float current_sp;
	float shaft_velocity_sp;
	float shaft_angle_sp;
	float sensor_offset;
	float zero_electric_angle;
	DQVoltage_s voltage;
	DQCurrent_s current;
	TorqueControlType torque_controller;
	MotionControlType controller;
} Motor_State_t;


/******************************************************************************/

/******************************************************************************/
float electricalAngle(uint8_t Motor_No);
float shaftVelocity(uint8_t Motor_No);
float shaftAngle(uint8_t Motor_No);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
	

#endif

