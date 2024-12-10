#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif
	
	
#include "stm32f4xx_hal.h"
#include "Timecount.h" 
/******************************************************************************/
typedef struct {
	float Tf; //!< Low pass filter time constant
	float y_prev; //!< filtered value in previous execution step 
	Interval_Timetick_t Lowpass_Timecount;
} LowPassFilter_t;

/******************************************************************************/
float LPF_velocity_(float x);
void LPF_init(uint8_t Motor_No);
float LPFoperator(LowPassFilter_t * LPF,float x);
/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif

