/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */
#define PWM_1 TIM_CHANNEL_1
#define PWM_2 TIM_CHANNEL_2
#define PWM_3 TIM_CHANNEL_3
#define PWM_4 TIM_CHANNEL_4

#define MAX_Period 400

#define TIM_TimeBaseStructure htim4

/* 以下两宏仅适用于定时器时钟源TIMxCLK=80MHz，预分频器为：7 的情况 */
#define SET_BASIC_TIM_PERIOD(T)     __HAL_TIM_SET_AUTORELOAD(&TIM_TimeBaseStructure, (T)*10000 - 1)    // 设置定时器的周期（1~1000ms）
#define GET_BASIC_TIM_PERIOD()      ((__HAL_TIM_GET_AUTORELOAD(&TIM_TimeBaseStructure)+1)/10000.0)     // 获取定时器的周期，单位ms

/* USER CODE END Private defines */

void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void Set_TIM3_PWM_CCRx(uint32_t Channel,float PWM_dc);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

