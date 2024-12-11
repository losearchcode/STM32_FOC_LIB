#ifndef INLINE_CS_LIB_H
#define INLINE_CS_LIB_H

#include "foc_utils.h" 
#include "FOCMotor.h"

typedef struct {
	ADC_HandleTypeDef *hadc;
	int channel;
	float gain;
	float I_offset;
} ADC_Pin_t;

typedef struct {
	ADC_Pin_t pinA;
	ADC_Pin_t pinB;
	ADC_Pin_t pinC;
	PhaseCurrent_s current;
} Current_Param_t;

/******************************************************************************/
void InlineCurrentSense_Init(uint8_t Motor_No,float _shunt_resistor, float _gain,int pinA_,int pinB_,int pinC_,ADC_HandleTypeDef *hadcn);

void getPhaseCurrents(uint8_t Motor_No);
/******************************************************************************/


#endif

