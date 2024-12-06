/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define FOC_Motor_Num 2
#define PID_ASSISTANT_EN
#define USE_PARK_ 1



#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "usart.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "stdio.h"
#include "protocol.h"
#include "Timecount.h"
//#include "struct.h"
#include "BLDCMotor.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//typedef  uint8_t u8;
//typedef  uint16_t u16;
//typedef  uint32_t u32;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern float Vel_Target;
extern float Angle_Target;
extern float target;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/******************************************************************************/
//���������ͣ�����ֻ��ѡһ������ʹ�õı�����Ϊ1����ʹ�õ�Ϊ0
#define M1_AS5600    1
#define M1_TLE5012B  0
//���ڴ���ֻ֧��2�ֱ���������һ�ڴ���֧�ָ��������
/******************************************************************************/
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
