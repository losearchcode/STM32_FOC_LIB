/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

//���Ŷ���
/*******************************************************/

#define FOC0_EN_PIN                  GPIO_PIN_9              
#define FOC0_EN_GPIO_PORT            GPIOB                     
#define FOC0_EN_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()


#define FOC1_EN_PIN                  GPIO_PIN_15              
#define FOC1_EN_GPIO_PORT            GPIOB                     
#define FOC1_EN_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()



/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  GPIO_PIN_SET
#define OFF GPIO_PIN_RESET

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN,~a)
#define FOC_EN(a)	HAL_GPIO_WritePin(FOC_EN_GPIO_PORT,FOC_EN_PIN,a)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			  //����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BSRR=(uint32_t)i << 16;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON			digitalLo(LED1_GPIO_PORT,LED1_PIN)


#define FOC0_ON		HAL_GPIO_WritePin(FOC0_EN_GPIO_PORT,FOC0_EN_PIN,ON)
#define FOC0_OFF	HAL_GPIO_WritePin(FOC0_EN_GPIO_PORT,FOC0_EN_PIN,OFF)

#define FOC1_ON		HAL_GPIO_WritePin(FOC1_EN_GPIO_PORT,FOC1_EN_PIN,ON)
#define FOC1_OFF	HAL_GPIO_WritePin(FOC1_EN_GPIO_PORT,FOC1_EN_PIN,OFF)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

