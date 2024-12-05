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
static TaskHandle_t KEY_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t PWM_DETECT_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t LCD_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t Closedloop_FOC_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t AS5600_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t EC11_Task_Handle = NULL;/* KEY任务句柄 */
static TaskHandle_t USART_Task_Handle = NULL;

static TaskHandle_t Protocol_Task_Handle = NULL;


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

//static double LED1_PWM_DC = 1.0;/* 定义一个创建信息返回值，默认为pdPASS */

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */


static void LED_Task(void* parameter);

static void KEY_Task(void* parameter);

static void PWM_DETECT_Task(void* parameter);

static void LCD_Task(void* parameter);

static void Closedloop_FOC_Task(void* parameter);

static void AS5600_Task(void* parameter);

static void EC11_Task(void* parameter);

static void USART_Task(void* parameter);

static void Protocol_Task(void* parameter);

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

//	printf("这是一个[野火]-STM32全系列开发板-FreeRTOS中断管理实验！\n");
//  printf("串口发送数据触发中断,任务处理数据!\n");
	
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
//					LED1(ON);
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
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&Led_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 LED_Task 任务成功!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )KEY_Task,  /* 任务入口函数 */
                        (const char*    )"KEY_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&KEY_Task_Handle);/* 任务控制块指针 */ 
												
  if(pdPASS == xReturn)
    printf("创建 KEY_Task 任务成功!\n\n");
	
//		xReturn = xTaskCreate((TaskFunction_t )PWM_DETECT_Task,  /* 任务入口函数 */
//                        (const char*    )"PWM_DETECT_Task",/* 任务名字 */
//                        (uint16_t       )512,  /* 任务栈大小 */
//                        (void*          )NULL,/* 任务入口函数参数 */
//                        (UBaseType_t    )2, /* 任务的优先级 */
//                        (TaskHandle_t*  )&PWM_DETECT_Task_Handle);/* 任务控制块指针 */ 
//  if(pdPASS == xReturn)
//    printf("创建 PWM_DETECT_Task 任务成功!\n\n");
	

	
		xReturn = xTaskCreate((TaskFunction_t )LCD_Task,  /* 任务入口函数 */
                        (const char*    )"LCD_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )&LCD_Show_Parameter,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&LCD_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 LCD_Task 任务成功!\n\n");
 
		xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_Task,  /* 任务入口函数 */
                        (const char*    )"Closedloop_FOC_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )5, /* 任务的优先级 */
                        (TaskHandle_t*  )&Closedloop_FOC_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 Closedloop_FOC_Task 任务成功!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )Protocol_Task,  /* 任务入口函数 */
                        (const char*    )"Protocol_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )4, /* 任务的优先级 */
                        (TaskHandle_t*  )&Protocol_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 Protocol_Task 任务成功!\n\n");	
												
	xReturn = xTaskCreate((TaskFunction_t )EC11_Task,  /* 任务入口函数 */
                        (const char*    )"EC11_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )3, /* 任务的优先级 */
                        (TaskHandle_t*  )&EC11_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 EC11_Task 任务成功!\n\n");	

//	xReturn = xTaskCreate((TaskFunction_t )USART_Task,  /* 任务入口函数 */
//                        (const char*    )"USART_Task",/* 任务名字 */
//                        (uint16_t       )512,  /* 任务栈大小 */
//                        (void*          )NULL,/* 任务入口函数参数 */
//                        (UBaseType_t    )2, /* 任务的优先级 */
//                        (TaskHandle_t*  )&USART_Task_Handle);/* 任务控制块指针 */ 
//  if(pdPASS == xReturn)
//    printf("创建 USART_Task 任务成功!\n\n");	
												
//		xReturn = xTaskCreate((TaskFunction_t )AS5600_Task,  /* 任务入口函数 */
//                        (const char*    )"AS5600_Task",/* 任务名字 */
//                        (uint16_t       )512,  /* 任务栈大小 */
//                        (void*          )NULL,/* 任务入口函数参数 */
//                        (UBaseType_t    )4, /* 任务的优先级 */
//                        (TaskHandle_t*  )&AS5600_Task_Handle);/* 任务控制块指针 */ 
//  if(pdPASS == xReturn)
//    printf("创建 AS5600_Task 任务成功!\n\n");
	
	

	
												
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
			vTaskDelay(100 + (uint8_t)(fabsf((float)LED1_Delay_time) * 100));
  }
}


/**********************************************************************
  * @ 函数名  ： KEY_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static uint8_t key_down_flag;
static void KEY_Task(void* parameter)
{	

  while (1)
  {
			if(HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_PIN) == 0 )
			{
				
				vTaskDelay(10);
				if(HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_PIN) == 0 && key_down_flag!=1)
				{
					key_down_flag = 1;
					{
//						pid_status=!pid_status;//取反状态
		  
//						#if defined(PID_ASSISTANT_EN) 
//									if (!pid_status)
//									{
//									set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
//									}
//									else
//									{
//									set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
//									}      
//						#endif
					}
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
				if(LED1_Delay_time<=-5)
				{
					LED1_Delay_time=-5;
					LED1_level_change_flag = ~LED1_level_change_flag; 
				}
			}	
  }
}

/**********************************************************************
  * @ 函数名  ： PWM_DETECT_Task
  * @ 功能说明： PWM_DETECT_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void PWM_DETECT_Task(void* parameter)
{	
  while (1)
  {
		vTaskDelay(1000);
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
	
	FOC_EN(ON);
  while (1)
  {
		if(FOC_Iint_Flag == 0)
		{
			FOC_Iint(&FOC_Init_Parameter[0], 0 , 12.4 , 7 , -1 ,&hi2c3,0.01,
									PID_Vel_Coefficient,PID_Angle_Coefficient);
			xLastWakeTime = xTaskGetTickCount();
						//设置速度环PID
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
			
			//设置速度
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
  * @ 函数名  ： LCD_Task
  * @ 功能说明： LCD_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
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
			LCD_Show_Parameter.AS5600_angle_Show = getAngle(FOC_Init_Parameter[0].Sensor_AS5600_);
			LCD_Show_Parameter.xCurrent_electricalAngle_Show = getMechanicalAngle(FOC_Init_Parameter[0].Sensor_AS5600_) ;
//			LCD_Show_Parameter.xCurrent_Vel_Show = getVelocity(FOC_Init_Parameter[0].Sensor_AS5600_);
			LCD_Show_Parameter.xElapsedTime_Show = FOC_Init_Parameter[0].Sensor_AS5600_->Vel_Interval_ts;
//			LCD_Show_Dynamic(parameter);
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
  * @ 函数名  ： AS5600_Task
  * @ 功能说明： AS5600_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void AS5600_Task(void* parameter)
{	
//	float angle;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(2); // 1000ms = 1s
	
	// 获取任务第一次启动时的滴答数
	xLastWakeTime = xTaskGetTickCount();
	
  while (1)
  {
//		angle = getAngle();
//		if(FOC_Iint_Flag)
//		Sensor_update(FOC_Init_Parameter[0].Sensor_AS5600_);
//		FOC_Init_Parameter.xCurrent_Sensor_Angle = angle;
//		
//		LCD_Show_Parameter.AS5600_angle_Show = FOC_Init_Parameter.xCurrent_Sensor_Angle;
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
  }
}
/**********************************************************************
  * @ 函数名  ： EC11_Task
  * @ 功能说明： EC11_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
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
  * @ 函数名  ： USART_Task
  * @ 功能说明： USART_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void USART_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(500);
	
  while (1)
  {
		{	
//			printf("EC11.EC11_Analyze_Last_Value :%d\n",EC11.EC11_Analyze_Last_Value);
			printf("%d\n",EC11.EC11_Analyze_Last_Value);
			vTaskDelay(xFrequency);
		}
  }
}

/**********************************************************************
  * @ 函数名  ： Protocol_Task
  * @ 功能说明： Protocol_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Protocol_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(10);
		
	/* 上位机初始化 */
	protocol_init();
	
	#if defined(PID_ASSISTANT_EN) 
    int temp = 2;    // 上位机需要整数参数，转换一下
    set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
		
		uint32_t temp_ = GET_BASIC_TIM_PERIOD()  ;                          // 设置定时器周期1~1000ms
		set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp_, 1);
		
//		pid_status= 1 ;
//		set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);
  #endif
	
  while (1)
  {
		{	
			/* 接收数据处理 */
		receiving_process();		
        
		#if defined(PID_ASSISTANT_EN) 
						temp = -Vel_Target;    // 上位机需要整数参数，转换一下
						set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &temp, 1);     // 给通道 1 发送目标值
						time_period_fun();
		#endif
		}   

		if( key_down_flag )
		{
//			pid_status=!pid_status;//取反状态
//		  
//		#if defined(PID_ASSISTANT_EN) 
//					if (!pid_status)
//					{
//					set_computer_value(SEND_START_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
//					}
//					else
//					{
//					set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);     // 同步上位机的启动按钮状态
//					}      
//		#endif
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
	
	/* I2C3初始化	*/
	MX_I2C3_Init();

	/* USART1初始化	*/
	MX_USART1_UART_Init();

	/* TIM4初始化	*/
	MX_TIM4_Init();


  
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
