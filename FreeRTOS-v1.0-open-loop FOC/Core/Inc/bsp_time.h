#ifndef __BSP_TIME_H
#define __BSP_TIME_H

#include "main.h"

#define PWM_1 TIM_CHANNEL_1
#define PWM_2 TIM_CHANNEL_2
#define PWM_3 TIM_CHANNEL_3
#define PWM_4 TIM_CHANNEL_4

#define MAX_Period 256*2


extern TIM_HandleTypeDef htim3;

void MX_TIM3_Init(void);
void Set_TIM3_PWM_CCRx(uint32_t Channel,float PWM_dc);

#endif

