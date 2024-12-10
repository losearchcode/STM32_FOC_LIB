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
static TaskHandle_t LCD_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t EC11_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t KEY_Task_Handle = NULL;/* KEY������ */
static TaskHandle_t Closedloop_FOC_Task_Handle = NULL;/* KEY������ */
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

float AREFA_SHOW;

uint32_t Protocol_Period = 200;


//float Vel_Target = 0.0;
//float Angle_Target = 0.0;
float target;
//static double LED1_PWM_DC = 1.0;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */

/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */

static void LED_Task(void* parameter);

static void KEY_Task(void* parameter);

static void EC11_Task(void* parameter);

static void LCD_Task(void* parameter);

static void Closedloop_FOC_Task(void* parameter);

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
                        (uint16_t       )128,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&Led_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� LED_Task ����ɹ�!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )KEY_Task,  /* ������ں��� */
                        (const char*    )"KEY_Task",/* �������� */
                        (uint16_t       )128,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&KEY_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� KEY_Task ����ɹ�!\n\n");
	
	
	xReturn = xTaskCreate((TaskFunction_t )EC11_Task,  /* ������ں��� */
                        (const char*    )"EC11_Task",/* �������� */
                        (uint16_t       )128,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )4, /* ��������ȼ� */
                        (TaskHandle_t*  )&EC11_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� EC11_Task ����ɹ�!\n\n");
	
	
	xReturn = xTaskCreate((TaskFunction_t )LCD_Task,  /* ������ں��� */
                        (const char*    )"LCD_Task",/* �������� */
                        (uint16_t       )256,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&LCD_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� LCD_Task ����ɹ�!\n\n");
 
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
                        (uint16_t       )256,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )4, /* ��������ȼ� */
                        (TaskHandle_t*  )&USART_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� USART_Task ����ɹ�!\n\n");	
	
	
												
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
		vTaskDelay(100 + (uint8_t)(fabsf((float)LED1_Delay_time) * 100));
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
				
				vTaskDelay(15);
				if(HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_PIN) == 0 && key_down_flag!=1)
				{
					key_down_flag = 1;
					{
//						pid_status=!pid_status;//ȡ��״̬
		  
//						#if defined(PID_ASSISTANT_EN) 
//									if (!pid_status)
//									{
//									set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);     // ͬ����λ����������ť״̬
//									}
//									else
//									{
//									set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);     // ͬ����λ����������ť״̬
//									}      
//						#endif
					}
					if(!LED1_level_change_flag)
						LED1_Delay_time++;
					else
						LED1_Delay_time--;
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
				if(LED1_Delay_time<=-5)
				{
					LED1_Delay_time=-5;
					LED1_level_change_flag = ~LED1_level_change_flag; 
				}
			}	
			else
			{
					vTaskDelay(20);
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
			GUI_Init(24);
//			LCD_Show_Static(parameter);
			GUI_Show_Static();
			vTaskDelay(50);
		}
		else
		{
			LCD_Show_Parameter.LED1_Delay_time_Show = LED1_Delay_time;
			LCD_Show_Parameter.AS5600_angle_Show = FOC_Para_M[0].Motor_State_.shaft_angle;
			LCD_Show_Parameter.xCurrent_electricalAngle_Show = FOC_Para_M[0].Motor_State_.electrical_angle ;
			LCD_Show_Parameter.xCurrent_Vel_Show = FOC_Para_M[0].Motor_State_.shaft_velocity;
			LCD_Show_Parameter.xElapsedTime_Show = FOC_Para_M[0].MagneticSensor_.AS5600_Vel_Timecount.xElapsedTime_t;

			if(GUI_Index_Current.Main_Menu_Index != GUI_Index_Current.Main_Menu_Last_Index)
			{
				GUI_Show_Static();
				GUI_Index_Current.Main_Menu_Last_Index =GUI_Index_Current.Main_Menu_Index;
				GUI_Index_Current.Submenu_Index = 0;
			}
			
			else
				GUI_Show_Dynamic();
			vTaskDelay(50);
		}
  }
}
/**********************************************************************
  * @ ������  �� EC11_Task
  * @ ����˵���� EC11_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void EC11_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(5);

	
  while (1)
  {
		if(EC11_Init_Flag == 0)
		{
			Encoder_EC11_Init(0,(uint8_t)xFrequency);
		}
		else
		{
			EC11.EC11_Analyze_Value = Encoder_EC11_Analyze(Encoder_EC11_Scan());
			if(EC11.EC11_Analyze_Value != 0 && EC11.EC11_Analyze_Value != EC11.EC11_Analyze_Last_Value)
				EC11.EC11_Analyze_Last_Value = EC11.EC11_Analyze_Value;
			
			EC11_GUI_Callback();

			vTaskDelay(xFrequency);
		}
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
	
	const TickType_t xFrequency = pdMS_TO_TICKS(5); // 1000ms = 1s
//	unsigned int count_i=0;
//	float Vel_;
	
	#if M1_AS5600
		/* I2C3��ʼ��	*/
//		MX_I2C3_Init();
		printf("AS5600\r\n");
	#elif M1_TLE5012B
		SPI2_Init_();              //TLE5012B
		printf("TLE5012B\r\n");
	#endif
	vTaskDelay(100);
	
	
	target=0;
	MagneticSensor_Init(0,&hi2c3);     //AS5600 or TLE5012B
	InlineCurrentSense_Init(0,0.01, 50,1,2,NOT_SET,&hadc1);
	Motor_init(0);
	Motor_initFOC(0,0,UNKNOWN);
	FOC_Para_M[0].Motor_State_.torque_controller = Type_dc_current;	
	//(0)Type_dc_current;  	(1)Type_foc_current;  (2)Type_voltage;
	FOC_Para_M[0].Motor_State_.controller = Type_torque;				
	//(0)Type_torque;  			(1)Type_velocity;  		(2)Type_angle;
  printf("Motor ready.\r\n");
	
  while (1)
  {
			move(0,target);
		
			loopFOC(0);
		
		
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
	
	const TickType_t xFrequency = pdMS_TO_TICKS(Protocol_Period);
	
		/* ��λ����ʼ�� */
	protocol_init();
	
	#if PID_ASSISTANT_EN
	Pro_Motor_EN = 1;
	set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 1); // ͬ����λ����������ť״̬
	
	float Pro_Target_Last;
	long temp = target;    // ��λ����Ҫ����������ת��һ��
	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
	
	uint32_t temp_ =Protocol_Period;                          // ���ö�ʱ������1~1000ms
	set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp_, 1);
	
	PID_init(0);
	
	float PID_temp_vel[3] = {FOC_Para_M[0].PID_Vel_.P,FOC_Para_M[0].PID_Vel_.I,FOC_Para_M[0].PID_Vel_.D};
	float PID_temp_angel[3] = {FOC_Para_M[0].PID_Angle_.P,FOC_Para_M[0].PID_Angle_.I,FOC_Para_M[0].PID_Angle_.D};
	set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, PID_temp_vel, 3);
	set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, PID_temp_angel, 3);
  #endif
	
	HAL_UART_Receive_IT(&huart1,&dr,1);
  while (1)
  {
		{	
						
        
		#if PID_ASSISTANT_EN
			if(Pro_Target_Last != set_point)
			{
						temp = set_point;    // ��λ����Ҫ����������ת��һ��
						set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
						set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &temp, 1);     // ��ͨ�� 1 ����Ŀ��ֵ
						Pro_Target_Last = set_point;
			}
			
						/* �������ݴ��� */
						receiving_process();
						time_period_fun();
			
		#else
				if((USART_RX_STA&0x8000)!=0)
				{
					switch(USART_RX_BUF[0])
					{
						case 'H':
							printf("Hello World!\r\n");
							break;
						case 'T':   //T6.28
							target=atof((const char *)(USART_RX_BUF+1));
							printf("RX=%.4f\r\n", target);
							break;
						case 'D':   //D
							FOC_EN(OFF);
							printf("OFF OK!\r\n");
							break;
						case 'E':   //E
							FOC_EN(ON);
							printf("ON OK!\r\n");
							break;
					}
					USART_RX_STA=0;
				}
				
				printf("target: %.4f rad/s\n\n",target);
				printf("velocity: %.4f rad/s\n\n",FOC_Para_M[0].Motor_State_.current.q);
//			printf("velocity: %.4f rad/s\n\n",FOC_Para_M[0].Motor_State_.shaft_velocity);
			
			
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
	
	/* I2C3������Closedloop_FOC_Task��ʼ��	*/
	MX_I2C3_Init();

	/* USART1��ʼ��	*/
	MX_USART1_UART_Init();

  /* ADC1��ʼ��	*/
  MX_ADC1_Init();
	
	



  
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
