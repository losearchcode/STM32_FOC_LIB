/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;

extern I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN Private defines */
#define I2C_P(obj) (Pthis = &obj)	

/* USER CODE END Private defines */

void MX_I2C2_Init(void);
void MX_I2C3_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct I2C_Private
{
	I2C_HandleTypeDef* I2Cx;
	uint16_t I2C_Add;
}I2C_Private;

typedef struct I2C_BUS
{
	//严禁使用该指针! It is strictly forbidden to use this pointer
	I2C_Private* Private;

	///////////////////////User//////////////////////////////////                   					    //用户API函数接口

	HAL_StatusTypeDef (*Write_Reg_Word)(uint8_t RegAddress, uint8_t Data);													//写寄存器函数,write register by I2C bus
	HAL_StatusTypeDef (*Read_Reg_Cont)(uint8_t RegAddress, uint8_t *pData, uint16_t Size);					//读寄存器函数,read register by I2C bus
																	
}I2C_BUS;

extern I2C_BUS* Pthis;

I2C_BUS Create_HI2C(I2C_HandleTypeDef* I2Cx,uint8_t Address);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

