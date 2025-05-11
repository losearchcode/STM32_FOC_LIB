/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "cmsis_os.h"
/* USER CODE BEGIN 0 */
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲，usart.h中定义长度
//接收状态
//bit15  接收完成标志
//bit14  接收到0x0D
//bit13~0  接收的字节数
unsigned short USART_RX_STA=0;       //接收状态标志

uint8_t dr;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	//HAL_UART_Receive_IT(&huart1,&dr,1);
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Usart_SendString(uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(&huart1,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}

///重定向c库函数printf到串口DEBUG_USART，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0XFFFF);	
	
	return (ch);
}

///重定向c库函数scanf到串口DEBUG_USART，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		
	int ch;
	HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, 0XFFFF);	
	return (ch);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
			uint32_t ulReturn;
			ulReturn=taskENTER_CRITICAL_FROM_ISR();
			unsigned char Res;
			Res = dr;	//读取接收到的字节
	
			if((USART_RX_STA&0x8000)==0)    //接收未完成
			{
				if(USART_RX_STA&0x4000)       //接收到了0x0d
				{
					if(Res!=0x0a)USART_RX_STA=0;   //接收错误，重新开始
					else 
					{
						USART_RX_STA|=0x8000;	       //接收完成
						USART_RX_BUF[USART_RX_STA&0X3FFF]='\0';   //最后一个字节放'0’，方便判断
					}
				}
				else //还没收到0x0D
				{	
					if(Res==0x0d)USART_RX_STA|=0x4000;
					else
					{
						USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
						USART_RX_STA++;
						if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;  //接收错误，重新开始  
					}		 
				}
			}
			
			if((USART_RX_STA&0x8000)!=0)
				{
					if(strncmp((char *)USART_RX_BUF, "0H", 2) == 0)
					{
						printf("-------------------------------------------------\r\n");
						printf("Hello World!\r\n");
						printf("target = %.4f\r\n", FOC_Para_M[0].Motor_State_.target);
						printf("P = %.4f\r\n", FOC_Para_M[0].PID_Vel_.P);
						printf("I = %.4f\r\n", FOC_Para_M[0].PID_Vel_.I);
						printf("D = %.4f\r\n", FOC_Para_M[0].PID_Vel_.D);
						printf("-------------------------------------------------\r\n");
					}
					else if(strncmp((char *)USART_RX_BUF, "0T", 2) == 0)
					{
						FOC_Para_M[0].Motor_State_.target = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[0].Motor_State_.target);
					}
					else if(strncmp((char *)USART_RX_BUF, "0P", 2) == 0)
					{
						FOC_Para_M[0].PID_Vel_.P = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[0].PID_Vel_.P);
					}
					else if(strncmp((char *)USART_RX_BUF, "0I", 2) == 0)
					{
						FOC_Para_M[0].PID_Vel_.I = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[0].PID_Vel_.I);
					}
					else if(strncmp((char *)USART_RX_BUF, "0D", 2) == 0)
					{
						FOC_Para_M[0].PID_Vel_.D = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[0].PID_Vel_.D);
					}
					else if(strncmp((char *)USART_RX_BUF, "0O", 2) == 0)
					{
						FOC0_ON;
						printf("ON OK!\r\n");
					}
					else if(strncmp((char *)USART_RX_BUF, "0F", 2) == 0)
					{
						FOC0_OFF;
						printf("OFF OK!\r\n");
					}
					
					if(strncmp((char *)USART_RX_BUF, "1H", 2) == 0)
					{
						printf("-------------------------------------------------\r\n");
						printf("Hello World!\r\n");
						printf("target = %.4f\r\n", FOC_Para_M[1].Motor_State_.target);
						printf("P = %.4f\r\n", FOC_Para_M[1].PID_Vel_.P);
						printf("I = %.4f\r\n", FOC_Para_M[1].PID_Vel_.I);
						printf("D = %.4f\r\n", FOC_Para_M[1].PID_Vel_.D);
						printf("-------------------------------------------------\r\n");
					}
					else if(strncmp((char *)USART_RX_BUF, "1T", 2) == 0)
					{
						FOC_Para_M[1].Motor_State_.target = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[1].Motor_State_.target);
					}
					else if(strncmp((char *)USART_RX_BUF, "1P", 2) == 0)
					{
						FOC_Para_M[1].PID_Vel_.P = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[1].PID_Vel_.P);
					}
					else if(strncmp((char *)USART_RX_BUF, "1I", 2) == 0)
					{
						FOC_Para_M[1].PID_Vel_.I = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[1].PID_Vel_.I);
					}
					else if(strncmp((char *)USART_RX_BUF, "1D", 2) == 0)
					{
						FOC_Para_M[1].PID_Vel_.D = atof((const char *)(USART_RX_BUF+2));
						printf("RX=%.4f\r\n", FOC_Para_M[1].PID_Vel_.D);
					}
					else if(strncmp((char *)USART_RX_BUF, "1O", 2) == 0)
					{
						FOC1_ON;
						printf("ON OK!\r\n");
					}
					else if(strncmp((char *)USART_RX_BUF, "1F", 2) == 0)
					{
						FOC1_OFF;
						printf("OFF OK!\r\n");
					}

//					switch(USART_RX_BUF[0])
//					{
//						case 'H':
//							printf("Hello World!\r\n");
//							break;
//						case 'T':   //T6.28
//							FOC_Para_M[0].Motor_State_.target = atof((const char *)(USART_RX_BUF+1));
//							printf("RX=%.4f\r\n", FOC_Para_M[0].Motor_State_.target);
//							break;
//						case 'D':   //D
//							
//							printf("OFF OK!\r\n");
//							break;
//						case 'E':   //E
//							
//							printf("ON OK!\r\n");
//							break;
//					}
					USART_RX_STA=0;
				}
			
			HAL_UART_Receive_IT(&huart1,&dr,1);  
				
			taskEXIT_CRITICAL_FROM_ISR(ulReturn);
    }
}

/* USER CODE END 1 */
