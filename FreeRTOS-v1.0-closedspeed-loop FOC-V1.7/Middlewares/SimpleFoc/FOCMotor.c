#include "FOCMotor.h"
#include "main.h"
#include "BLDCMotor.h"

/******************************************************************************/
// shaft angle calculation
float shaftAngle(uint8_t Motor_No)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_angle;
  return FOC_Para_M[Motor_No].Motor_Properties_.Sensor_DIR*getAngle(Motor_No) - FOC_Para_M[Motor_No].Motor_State_.sensor_offset;
}
// shaft velocity calculation
float shaftVelocity(uint8_t Motor_No)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_velocity;
  return FOC_Para_M[Motor_No].Motor_Properties_.Sensor_DIR*LPF_velocity(getVelocity(Motor_No));
}
/******************************************************************************/
float electricalAngle(uint8_t Motor_No)
{
  return _normalizeAngle((FOC_Para_M[Motor_No].Motor_State_.shaft_angle + FOC_Para_M[Motor_No].Motor_State_.sensor_offset) * FOC_Para_M[Motor_No].Motor_Properties_.Motor_Pole_Pairs - FOC_Para_M[Motor_No].Motor_State_.zero_electric_angle);
}
/******************************************************************************/


