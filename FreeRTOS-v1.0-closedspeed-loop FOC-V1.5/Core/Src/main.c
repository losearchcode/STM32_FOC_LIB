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

/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
 /* 创建任务句柄 */
static TaskHandle_t AppTaskCreate_Handle;

static TaskHandle_t Led_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t Closedloop_FOC_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t USART_Task_Handle = NULL;




/********************************** 内核对象句柄 *********************************/
/*
 * 互斥量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */

/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */

static int LED1_Delay_time = 0;/* 定义一个创建信息返回值，默认为pdPASS */
static uint8_t LED1_level_change_flag = 0;

float Vel_Target = 0.0;
float Angle_Target = 0.0;
float target;
//static double LED1_PWM_DC = 1.0;/* 定义一个创建信息返回值，默认为pdPASS */

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */

static void LED_Task(void* parameter);

static void Closedloop_FOC_Task(void* parameter);

static void USART_Task(void* parameter);

static void BSP_Init(void);/* 用于初始化板载相关资源 */

/******************************* 宏定义 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些宏定义。
 */
/**
  * @brief  The application entry point.
  * @retval int
  */
	
	
int main(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */

  /* 开发板硬件初始化 */
  BSP_Init();
	
   /* 创建 AppTaskCreate 任务 */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* 任务入口函数 */
                        (const char*    )"AppTaskCreate",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* 任务控制块指针 */ 
															
	if(pdFAIL != xReturn)/* 创建成功 */
    vTaskStartScheduler();   /* 启动任务，开启调度 */
  
    while(1)
		{
			
		}
}

/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
	
  taskENTER_CRITICAL();           //进入临界区

	
	  /* 创建LED_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )LED_Task,  /* 任务入口函数 */
                        (const char*    )"LED_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )2, /* 任务的优先级 */
                        (TaskHandle_t*  )&Led_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 LED_Task 任务成功!\n\n");
 
		xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_Task,  /* 任务入口函数 */
                        (const char*    )"Closedloop_FOC_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )5, /* 任务的优先级 */
                        (TaskHandle_t*  )&Closedloop_FOC_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 Closedloop_FOC_Task 任务成功!\n\n");
	
												
	xReturn = xTaskCreate((TaskFunction_t )USART_Task,  /* 任务入口函数 */
                        (const char*    )"USART_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&USART_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 USART_Task 任务成功!\n\n");	
	
	
												
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
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
  * @ 函数名  ： Closedloop_FOC_Task
  * @ 功能说明： Closedloop_FOC_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Closedloop_FOC_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(3); // 1000ms = 1s
	unsigned int count_i=0;
	float Vel_;
	
	#if M1_AS5600
		/* I2C3初始化	*/
		MX_I2C3_Init();               //AS5600
		printf("AS5600\r\n");
	#elif M1_TLE5012B
		SPI2_Init_();              //TLE5012B
		printf("TLE5012B\r\n");
	#endif
		vTaskDelay(1000);
	
	
	target=0;
	MagneticSensor_Init(0,&hi2c3);     //AS5600 or TLE5012B
	Motor_init(0);
	Motor_initFOC(0);
	PID_init(0);                //PID参数设置 在init函数里
  printf("Motor ready.\r\n");
	
	FOC_EN(ON);
  while (1)
  {
			move(0,target);
		
			loopFOC(0);
		
		
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
  
}


/**********************************************************************
  * @ 函数名  ： USART_Task
  * @ 功能说明： USART_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void USART_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(200);
	
	float Vel_Target_Last;
		/* 上位机初始化 */
	protocol_init();
	
	#if defined(PID_ASSISTANT_EN) 
	int temp = Vel_Target;    // 上位机需要整数参数，转换一下
	set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
	HAL_UART_Receive_IT(&huart1,&dr,1);
  #endif
	
	
  while (1)
  {
		{	
						
        
		#if defined(PID_ASSISTANT_EN) 
			if(Vel_Target_Last != Vel_Target)
			{
						temp = Vel_Target;    // 上位机需要整数参数，转换一下
						set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
						Vel_Target_Last = Vel_Target;
			}
						/* 接收数据处理 */
						receiving_process();
						time_period_fun();
		#endif
			
		}
		vTaskDelay(xFrequency);
	}
			
}
 


/***********************************************************************
  * @ 函数名  ： SystemClock_Config
  * @ 功能说明： 系统时钟源配置
  * @ 参数    ：   
  * @ 返回值  ： 无
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
  * @ 函数名  ： BSP_Init
  * @ 功能说明： 板级外设初始化，所有板子上的初始化均可放在这个函数里面
  * @ 参数    ：   
  * @ 返回值  ： 无
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32中断优先级分组为4，即4bit都用来表示抢占优先级，范围为：0~15
	 * 优先级分组只需要分组一次即可，以后如果有其他的任务需要用到中断，
	 * 都统一用这个优先级分组，千万不要再分组，切忌。
	 */
	
	/* HAL_初始化含 tick、pendsv优先级设置和NVIC分组 */
	HAL_Init();
	
	/* 初始化系统时钟 */
	SystemClock_Config();

	
	/* GPIO 初始化 */
	MX_GPIO_Init();
	
	/* TIM3初始化	*/
	MX_TIM3_Init();
	
//	/* I2C3初始化	*/
//	MX_I2C3_Init();

	/* USART1初始化	*/
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
