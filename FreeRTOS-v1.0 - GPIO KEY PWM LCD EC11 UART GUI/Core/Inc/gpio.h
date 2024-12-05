/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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

#define LED1_PIN                  GPIO_PIN_13               
#define LED1_GPIO_PORT            GPIOC                     
#define LED1_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

#define KEY1_PIN                  GPIO_PIN_0               
#define KEY1_GPIO_PORT            GPIOA                    
#define KEY1_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

#define EC11_KEY_PIN                GPIO_PIN_13               
#define EC11_KEY_GPIO_PORT          GPIOB                    
#define EC11_KEY_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define EC11_A_PIN                  GPIO_PIN_12               
#define EC11_A_GPIO_PORT            GPIOB                    
#define EC11_A_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

#define EC11_B_PIN                  GPIO_PIN_14               
#define EC11_B_GPIO_PORT            GPIOB                    
#define EC11_B_CLK_ENABLE()    			__GPIOB_CLK_ENABLE()

/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  GPIO_PIN_RESET
#define OFF GPIO_PIN_SET

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN,a)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			  //����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BSRR=(uint32_t)i << 16;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON			digitalLo(LED1_GPIO_PORT,LED1_PIN)
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

