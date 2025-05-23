#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "main.h"
	
	
/******************************************************************************/
/**
 *  Motiron control type
 */
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

extern TorqueControlType torque_controller;
extern MotionControlType controller;
/******************************************************************************/
extern float shaft_angle;//!< current motor angle
extern float electrical_angle;
extern float shaft_velocity;
extern float current_sp;
extern float shaft_velocity_sp;
extern float shaft_angle_sp;
extern DQVoltage_s voltage;
extern DQCurrent_s current;

extern float sensor_offset;
extern float zero_electric_angle;
/******************************************************************************/
float shaftAngle(void);
float shaftVelocity(void);
float electricalAngle(void);
/******************************************************************************/
#ifdef __cplusplus
}
#endif
	

#endif

