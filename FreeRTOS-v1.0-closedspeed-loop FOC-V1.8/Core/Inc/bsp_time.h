#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#include "main.h"


#define MAX_Period 5000


extern TIM_HandleTypeDef htim3;

void MX_TIM3_Init(void);
void Set_TIM3_PWM_CCRx(uint32_t Channel,float PWM_dc);

#endif

