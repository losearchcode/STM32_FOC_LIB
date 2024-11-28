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
//引脚定义
/*******************************************************/

#define LED1_PIN                 	 	GPIO_PIN_13               
#define LED1_GPIO_PORT           	 	GPIOC                     
#define LED1_GPIO_CLK_ENABLE()    	__GPIOC_CLK_ENABLE()

#define FOC_EN_PIN                  GPIO_PIN_2               
#define FOC_EN_GPIO_PORT            GPIOB                     
#define FOC_EN_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

#define KEY1_PIN                 	 	GPIO_PIN_0               
#define KEY1_GPIO_PORT           		GPIOA                    
#define KEY1_GPIO_CLK_ENABLE()   		__GPIOA_CLK_ENABLE()

#define KEY2_PIN                 	 	GPIO_PIN_3               
#define KEY2_GPIO_PORT           		GPIOA                    
#define KEY2_GPIO_CLK_ENABLE()   		__GPIOA_CLK_ENABLE()

/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  GPIO_PIN_SET
#define OFF GPIO_PIN_RESET

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN,~a)
#define FOC_EN(a)	HAL_GPIO_WritePin(FOC_EN_GPIO_PORT,FOC_EN_PIN,a)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			  //设置为高电平		
#define digitalLo(p,i)			{p->BSRR=(uint32_t)i << 16;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态


/* 定义控制IO的宏 */
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

