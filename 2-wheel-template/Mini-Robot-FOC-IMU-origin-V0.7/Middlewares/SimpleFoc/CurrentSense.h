#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H

/******************************************************************************/
#include "foc_utils.h" 
#include "FOCMotor.h"

/******************************************************************************/
float getDCCurrent(uint8_t Motor_No,float motor_electrical_angle);
DQCurrent_s getFOCCurrents(uint8_t Motor_No,float angle_el);
/******************************************************************************/


#endif

