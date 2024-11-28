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
static TaskHandle_t KEY_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t PWM_DETECT_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t LCD_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t Openloop_FOC_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t AS5600_Task_Handle = NULL;/* KEY������ */



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

static BaseType_t LED1_Delay_time = 0;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
static uint8_t LED1_level_change_flag = 0;

//static double LED1_PWM_DC = 1.0;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */

/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */


static void LED_Task(void* parameter);

static void KEY_Task(void* parameter);

static void PWM_DETECT_Task(void* parameter);

static void LCD_Task(void* parameter);

static void Openloop_FOC_Task(void* parameter);

static void AS5600_Task(void* parameter);

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

//	printf("����һ��[Ұ��]-STM32ȫϵ�п�����-FreeRTOS�жϹ���ʵ�飡\n");
//  printf("���ڷ������ݴ����ж�,����������!\n");
	
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
//					LED1(ON);
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
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&Led_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
//    printf("���� LED_Task ����ɹ�!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )KEY_Task,  /* ������ں��� */
                        (const char*    )"KEY_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&KEY_Task_Handle);/* ������ƿ�ָ�� */ 
												
  if(pdPASS == xReturn)
//    printf("���� KEY_Task ����ɹ�!\n\n");
	
		xReturn = xTaskCreate((TaskFunction_t )PWM_DETECT_Task,  /* ������ں��� */
                        (const char*    )"PWM_DETECT_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )2, /* ��������ȼ� */
                        (TaskHandle_t*  )&PWM_DETECT_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
//    printf("���� KEY_Task ����ɹ�!\n\n");
	

	
		xReturn = xTaskCreate((TaskFunction_t )LCD_Task,  /* ������ں��� */
                        (const char*    )"LCD_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )&LCD_Show_Parameter,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&LCD_Task_Handle);/* ������ƿ�ָ�� */ 
//  if(pdPASS == xReturn)
//    printf("���� KEY_Task ����ɹ�!\n\n");
 
		xReturn = xTaskCreate((TaskFunction_t )Openloop_FOC_Task,  /* ������ں��� */
                        (const char*    )"Openloop_FOC_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )5, /* ��������ȼ� */
                        (TaskHandle_t*  )&Openloop_FOC_Task_Handle);/* ������ƿ�ָ�� */ 
//  if(pdPASS == xReturn)
//    printf("���� KEY_Task ����ɹ�!\n\n");
												
												
												
		xReturn = xTaskCreate((TaskFunction_t )AS5600_Task,  /* ������ں��� */
                        (const char*    )"AS5600_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )4, /* ��������ȼ� */
                        (TaskHandle_t*  )&AS5600_Task_Handle);/* ������ƿ�ָ�� */ 
//  if(pdPASS == xReturn)
//    printf("���� KEY_Task ����ɹ�!\n\n");
												
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
			vTaskDelay(100 + LED1_Delay_time * 100);
  }
}


/**********************************************************************
  * @ ������  �� KEY_Task
  * @ ����˵���� LED_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void KEY_Task(void* parameter)
{	
	static uint8_t key_down_flag;
  while (1)
  {
			if(HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_PIN) == 0 )
			{
				
				vTaskDelay(10);
				if(HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_PIN) == 0 && key_down_flag!=1)
				{
					key_down_flag = 1;
					if(!LED1_level_change_flag)
						LED1_Delay_time++;
					else
						LED1_Delay_time--;
					
//					LCD_refresh_flag = 1;
				}
				if(HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_PIN) == 1)
				{
					key_down_flag=0;
				}
				if(LED1_Delay_time>=5)
				{
					LED1_Delay_time=5;
					LED1_level_change_flag = ~LED1_level_change_flag; 
				}
				if(LED1_Delay_time<=0)
				{
					LED1_Delay_time=0;
					LED1_level_change_flag = ~LED1_level_change_flag; 
				}
			}	
  }
}

/**********************************************************************
  * @ ������  �� PWM_DETECT_Task
  * @ ����˵���� PWM_DETECT_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void PWM_DETECT_Task(void* parameter)
{	
  while (1)
  {
		vTaskDelay(1000);
  }
}

/**********************************************************************
  * @ ������  �� Openloop_FOC_Task
  * @ ����˵���� Openloop_FOC_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void Openloop_FOC_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
//	TickType_t xLastWakeTime_t;
	const TickType_t xFrequency = pdMS_TO_TICKS(3); // 1000ms = 1s
	
	// ��ȡ�����һ������ʱ�ĵδ���
	xLastWakeTime = xTaskGetTickCount();
	
//	xLastWakeTime_t = xLastWakeTime;
	
	FOC_Init_Parameter.power_supply_t = 11.4;
	FOC_Init_Parameter.Motor_PP = 7;
	FOC_Init_Parameter.Sensor_DIR = -1;
	float Kp = 0.086;
	
	FOC_EN(ON);
  while (1)
  {
		if(FOC_Iint_Flag == 0)
		{
			FOC_Iint();
			First_Time_Laod(&FOC_Task_Timecount,xLastWakeTime);
//			xLastWakeTime_t = xTaskGetTickCount();
		}
		else
		{
//			// ��ȡ��ǰ�δ���
//			TickType_t xCurrentTime = xTaskGetTickCount();

			// �������ϴλ���������ʱ����
//			TickType_t xElapsedTime = xCurrentTime - xLastWakeTime;
			TickType_t xElapsedTime = Get_Interval_Timetick(&FOC_Task_Timecount);
		
			FOC_Init_Parameter.xCurrent_Sensor_Angle = getAngle();
			
			float Sensor_Angle = FOC_Init_Parameter.xCurrent_Sensor_Angle;
			int Sensor_DIR = FOC_Init_Parameter.Sensor_DIR;
			setTorque(Kp*(LED1_Delay_time*1.256-
										Sensor_DIR*Sensor_Angle)*180/PI,
										FOC_Init_Parameter.xCurrent_electricalAngle);
			
			
			LCD_Show_Parameter.xElapsedTime_Show = xElapsedTime;
//			xLastWakeTime = xCurrentTime;
			
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
  }
}

/**********************************************************************
  * @ ������  �� LCD_Task
  * @ ����˵���� LCD_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void LCD_Task(void* parameter)
{	
  while (1)
  {
		if(LCD_init_flag == 0)
		{
			LCD_Init();
			LCD_Show_Static(parameter);
			vTaskDelay(50);
		}
		else
		{
			LCD_Show_Parameter.LED1_Delay_time_Show = LED1_Delay_time;
			LCD_Show_Parameter.AS5600_angle_Show = FOC_Init_Parameter.xCurrent_Sensor_Angle;
			LCD_Show_Parameter.xCurrent_electricalAngle_Show = FOC_Init_Parameter.xCurrent_electricalAngle;
			LCD_Show_Dynamic(parameter);
			vTaskDelay(50);
		}
  }
}

/**********************************************************************
  * @ ������  �� LCD_Task
  * @ ����˵���� LCD_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void AS5600_Task(void* parameter)
{	
	float angle;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(2); // 1000ms = 1s
	
	// ��ȡ�����һ������ʱ�ĵδ���
	xLastWakeTime = xTaskGetTickCount();
	
  while (1)
  {
//		angle = getAngle();
//		
//		FOC_Init_Parameter.xCurrent_Sensor_Angle = angle;
//		
//		LCD_Show_Parameter.AS5600_angle_Show = FOC_Init_Parameter.xCurrent_Sensor_Angle;
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  
}



/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
