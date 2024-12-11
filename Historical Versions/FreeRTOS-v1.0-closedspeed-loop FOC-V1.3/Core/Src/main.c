/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/**************************** ������ ********************************/
/* 
 * ��������һ��ָ�룬����ָ��һ�����񣬵����񴴽���֮�����;�����һ��������
 * �Ժ�����Ҫ��������������Ҫͨ�������������������������������Լ�����ô
 * ����������ΪNULL��
 */
 /* ���������� */
static TaskHandle_t AppTaskCreate_Handle;

static TaskHandle_t Led_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t Closedloop_FOC_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t AS5600_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t USART_Task_Handle = NULL;




/********************************** �ں˶����� *********************************/
/*
 * ����������Ϣ���У��¼���־�飬�����ʱ����Щ�������ں˵Ķ���Ҫ��ʹ����Щ�ں�
 * ���󣬱����ȴ����������ɹ�֮��᷵��һ����Ӧ�ľ����ʵ���Ͼ���һ��ָ�룬������
 * �ǾͿ���ͨ��������������Щ�ں˶���
 *
 * �ں˶���˵���˾���һ��ȫ�ֵ����ݽṹ��ͨ����Щ���ݽṹ���ǿ���ʵ��������ͨ�ţ�
 * �������¼�ͬ���ȸ��ֹ��ܡ�������Щ���ܵ�ʵ��������ͨ��������Щ�ں˶���ĺ���
 * ����ɵ�
 * 
 */

/******************************* ȫ�ֱ������� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩȫ�ֱ�����
 */

static int LED1_Delay_time = 0;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
static uint8_t LED1_level_change_flag = 0;

float Vel_Target = 0.0;
float Angle_Target = 0.0;

//static double LED1_PWM_DC = 1.0;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */

/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */


static void LED_Task(void* parameter);

static void Closedloop_FOC_Task(void* parameter);

static void AS5600_Task(void* parameter);

static void USART_Task(void* parameter);

static void BSP_Init(void);/* ���ڳ�ʼ�����������Դ */

/******************************* �궨�� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩ�궨�塣
 */
/**
  * @brief  The application entry point.
  * @retval int
  */
	
	
int main(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */

  /* ������Ӳ����ʼ�� */
  BSP_Init();
	
   /* ���� AppTaskCreate ���� */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* ������ں��� */
                        (const char*    )"AppTaskCreate",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */ 
															
	if(pdFAIL != xReturn)/* �����ɹ� */
    vTaskStartScheduler();   /* �������񣬿������� */
  
    while(1)
		{
			
		}
}

/***********************************************************************
  * @ ������  �� AppTaskCreate
  * @ ����˵���� Ϊ�˷���������е����񴴽����������������������
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
	
  taskENTER_CRITICAL();           //�����ٽ���

	
	  /* ����LED_Task���� */
  xReturn = xTaskCreate((TaskFunction_t )LED_Task,  /* ������ں��� */
                        (const char*    )"LED_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )2, /* ��������ȼ� */
                        (TaskHandle_t*  )&Led_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� LED_Task ����ɹ�!\n\n");
 
		xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_Task,  /* ������ں��� */
                        (const char*    )"Closedloop_FOC_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )5, /* ��������ȼ� */
                        (TaskHandle_t*  )&Closedloop_FOC_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� Closedloop_FOC_Task ����ɹ�!\n\n");
	
												
	xReturn = xTaskCreate((TaskFunction_t )USART_Task,  /* ������ں��� */
                        (const char*    )"USART_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&USART_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� USART_Task ����ɹ�!\n\n");	
	
	xReturn = xTaskCreate((TaskFunction_t )AS5600_Task,  /* ������ں��� */
                        (const char*    )"AS5600_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )5, /* ��������ȼ� */
                        (TaskHandle_t*  )&AS5600_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� AS5600_Task ����ɹ�!\n\n");	
	
												
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}

/**********************************************************************
  * @ ������  �� LED_Task
  * @ ����˵���� LED_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void LED_Task(void* parameter)
{	
  while (1)
  {
			LED1_TOGGLE;
//			vTaskDelay(100 + (uint8_t)(fabsf((float)LED1_Delay_time) * 100));
		vTaskDelay(100 + (uint8_t)(fabsf((float)Vel_Target) * 100));
  }
}


/**********************************************************************
  * @ ������  �� Closedloop_FOC_Task
  * @ ����˵���� Closedloop_FOC_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void Closedloop_FOC_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(3); // 1000ms = 1s
	
	FOC_EN(ON);
  while (1)
  {
		if(FOC_Iint_Flag == 0)
		{
			FOC_Iint(&FOC_Init_Parameter[0], 0 , 12.4 , 7 , -1 ,&hi2c3,0.01,
									PID_Vel_Coefficient,PID_Angle_Coefficient);
			xLastWakeTime = xTaskGetTickCount();
						//�����ٶȻ�PID
			DFOC_M0_SET_VEL_PID(&FOC_Init_Parameter[0],0.002675,0.003,0.0,0);
			DFOC_M0_SET_ANGLE_PID(&FOC_Init_Parameter[0],0.09,0.0005,0.0,0);
		}
		else
		{
			TickType_t xElapsedTime = Get_Interval_Timetick(&FOC_Init_Parameter[0].FOC_Task_Timecount);
			Vel_Target = _constrain(Vel_Target,-100,100);
			
			if(fabsf(Vel_Target)<15)
			{
				PID_Vel_Loop_Parameter[0].P =(float)((double)0.002675/2.0);
				PID_Vel_Loop_Parameter[0].I =(float)((double)0.003/(double)2.0);
				PID_Vel_Loop_Parameter[0].D = 0.0;
			}
			else
			{
				DFOC_M0_SET_VEL_PID(&FOC_Init_Parameter[0],0.002675/1.5,0.003/1.25,0.0,0);
			}
			
			//�����ٶ�
			DFOC_M0_setVelocity(&FOC_Init_Parameter[0],Vel_Target);
//			DFOC_M0_set_Velocity_Angle(&FOC_Init_Parameter[0],LED1_Delay_time*1.28);
//			DFOC_M0_set_Force_Angle(&FOC_Init_Parameter[0],Angle_Target);
//			Sensor_update(&Sensor_AS5600_Parameter[0]);
			
			LCD_Show_Parameter.xElapsedTime_Show = xElapsedTime;
			
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
  
}
}

/**********************************************************************
  * @ ������  �� AS5600_Task
  * @ ����˵���� AS5600_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void AS5600_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(2); // 1000ms = 1s
	
	// ��ȡ�����һ������ʱ�ĵδ���
	xLastWakeTime = xTaskGetTickCount();
	
  while (1)
  {
//		angle = getAngle();
//		if(FOC_Iint_Flag)
		Sensor_update(FOC_Init_Parameter[0].Sensor_AS5600_);
//		FOC_Init_Parameter.xCurrent_Sensor_Angle = angle;
//		
//		LCD_Show_Parameter.AS5600_angle_Show = FOC_Init_Parameter.xCurrent_Sensor_Angle;
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
  }
}

/**********************************************************************
  * @ ������  �� USART_Task
  * @ ����˵���� USART_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void USART_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(20);
	
	float Vel_Target_Last;
		/* ��λ����ʼ�� */
	protocol_init();
	
	#if defined(PID_ASSISTANT_EN) 
	int temp = Vel_Target;    // ��λ����Ҫ����������ת��һ��
	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
		
  #endif
	
	HAL_UART_Receive_IT(&huart1,&dr,1);
  while (1)
  {
		{	
						/* �������ݴ��� */
				receiving_process();
        
		#if defined(PID_ASSISTANT_EN) 
//			if(Vel_Target_Last != Vel_Target)
//			{
//						temp = Vel_Target;    // ��λ����Ҫ����������ת��һ��
//						set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
//						Vel_Target_Last = Vel_Target;
//			}
						
						time_period_fun();
		#endif
			
		}
		vTaskDelay(xFrequency);
	}
			
}
 


/***********************************************************************
  * @ ������  �� SystemClock_Config
  * @ ����˵���� ϵͳʱ��Դ����
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/***********************************************************************
  * @ ������  �� BSP_Init
  * @ ����˵���� �弶�����ʼ�������а����ϵĳ�ʼ�����ɷ��������������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32�ж����ȼ�����Ϊ4����4bit��������ʾ��ռ���ȼ�����ΧΪ��0~15
	 * ���ȼ�����ֻ��Ҫ����һ�μ��ɣ��Ժ������������������Ҫ�õ��жϣ�
	 * ��ͳһ��������ȼ����飬ǧ��Ҫ�ٷ��飬�мɡ�
	 */
	
	/* HAL_��ʼ���� tick��pendsv���ȼ����ú�NVIC���� */
	HAL_Init();
	
	/* ��ʼ��ϵͳʱ�� */
	SystemClock_Config();

	
	/* GPIO ��ʼ�� */
	MX_GPIO_Init();
	
	/* TIM3��ʼ��	*/
	MX_TIM3_Init();
	
	/* I2C3��ʼ��	*/
	MX_I2C3_Init();

	/* USART1��ʼ��	*/
	MX_USART1_UART_Init();



  
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
