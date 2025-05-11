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

static TaskHandle_t MPU6050_Task_Handle = NULL;/* KEY任务句柄 */

static TaskHandle_t USART1_Task_Handle = NULL;

static TaskHandle_t USART3_Task_Handle = NULL;

static TaskHandle_t Closedloop_FOC_0_Init_Task_Handle = NULL;

static TaskHandle_t Closedloop_FOC_1_Init_Task_Handle = NULL;

static TaskHandle_t Closedloop_FOC_Task_Handle = NULL;


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
 **********************************************************************************************/

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */

static void MPU6050_Task(void* parameter);

static void USART1_Task(void* parameter);

static void USART3_Task(void* parameter);

static void Closedloop_FOC_0_Init_Task(void* parameter);

static void Closedloop_FOC_1_Init_Task(void* parameter);

static void Closedloop_FOC_Task(void* parameter);


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
//  xReturn = xTaskCreate((TaskFunction_t )MPU6050_Task,  /* 任务入口函数 */
//                        (const char*    )"MPU6050_Task",/* 任务名字 */
//                        (uint16_t       )256,  /* 任务栈大小 */
//                        (void*          )NULL,/* 任务入口函数参数 */
//                        (UBaseType_t    )4, /* 任务的优先级 */
//                        (TaskHandle_t*  )&MPU6050_Task_Handle);/* 任务控制块指针 */ 
//  if(pdPASS == xReturn)
//    printf("创建 LED_Task 任务成功!\n\n");
	

		  /* 创建LED_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_0_Init_Task,  /* 任务入口函数 */
                        (const char*    )"Closedloop_FOC_0_Init_Task",/* 任务名字 */
                        (uint16_t       )256,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )6, /* 任务的优先级 */
                        (TaskHandle_t*  )&Closedloop_FOC_0_Init_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 Closedloop_FOC_0_Init_Task 任务成功!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_1_Init_Task,  /* 任务入口函数 */
							(const char*    )"Closedloop_FOC_1_Init_Task",/* 任务名字 */
							(uint16_t       )256,  /* 任务栈大小 */
							(void*          )NULL,/* 任务入口函数参数 */
							(UBaseType_t    )6, /* 任务的优先级 */
							(TaskHandle_t*  )&Closedloop_FOC_1_Init_Task_Handle);/* 任务控制块指针 */ 
	if(pdPASS == xReturn)
		printf("创建 Closedloop_FOC_1_Init_Task 任务成功!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_Task,  /* 任务入口函数 */
							(const char*    )"Closedloop_FOC_0_Task",/* 任务名字 */
							(uint16_t       )256,  /* 任务栈大小 */
							(void*          )NULL,/* 任务入口函数参数 */
							(UBaseType_t    )5, /* 任务的优先级 */
							(TaskHandle_t*  )&Closedloop_FOC_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 Closedloop_FOC_0_Task 任务成功!\n\n");
	

	
	xReturn = xTaskCreate((TaskFunction_t )USART1_Task,  /* 任务入口函数 */
                        (const char*    )"USART1_Task",/* 任务名字 */
                        (uint16_t       )256,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )4, /* 任务的优先级 */
                        (TaskHandle_t*  )&USART1_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 USART1_Task 任务成功!\n\n");	
	
	xReturn = xTaskCreate((TaskFunction_t )USART3_Task,  /* 任务入口函数 */
                        (const char*    )"USART3_Task",/* 任务名字 */
                        (uint16_t       )256,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )4, /* 任务的优先级 */
                        (TaskHandle_t*  )&USART3_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    printf("创建 USART3_Task 任务成功!\n\n");	
	
	
												
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}

/**********************************************************************
  * @ 函数名  ： LED_Task
  * @ 功能说明： LED_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/

static void MPU6050_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(20);
	uint8_t MPU6050_ID;
	uint8_t init_count = 0;
	
	MPU6050_init(&hi2c2);
	MPU6050_ID = MPU6050_ID_Get();
	printf("MPU6050_ID:%x\r\n",MPU6050_ID);
	
	
	
  while (1)
  {
		
		MPU6050_Get_Angle(&MPU6050_value,&MPU6050_raw_t);
		if(init_count<=80)
		{
			MPU6050_value_init.pitch += MPU6050_value.pitch ;
			MPU6050_value_init.roll += MPU6050_value.roll ;
			MPU6050_value_init.yaw += MPU6050_value.yaw ;
			init_count++;
		}
		if(init_count == 80)
		{
			MPU6050_value_init.pitch =MPU6050_value.pitch;
			MPU6050_value_init.roll =MPU6050_value.roll;
			MPU6050_value_init.yaw =MPU6050_value.yaw;
			init_count++;
		}
		
	
//			vTaskDelay(100 + (uint8_t)(fabsf((float)LED1_Delay_time) * 100));
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
  }
}

/**********************************************************************
  * @ 函数名  ： Closedloop_FOC_0_Init_Task
  * @ 功能说明： Closedloop_FOC_0_Init_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
uint8_t FOC_Init_Flag[2] = {0,0};
static void Closedloop_FOC_0_Init_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(4); // 1000ms = 1s
//	unsigned int count_i=0;
//	float Vel_;
	
	#if M1_AS5600
		/* I2C3初始化	*/
//		MX_I2C3_Init();
		printf("AS5600\r\n");
	#elif M1_TLE5012B
		SPI2_Init_();              //TLE5012B
		printf("TLE5012B\r\n");
	#endif
	//vTaskDelay(100);
	
  while (1)
  {
			FOC_Para_M[0].Motor_State_.target = 6.28;
			MagneticSensor_Init(0,&hi2c3);     //AS5600 or TLE5012B
			InlineCurrentSense_Init(0,0.01, 50,6,7,NOT_SET,&hadc1);
			Motor_init(0,&htim4);
			Motor_initFOC(0,0,UNKNOWN);
			
			FOC_Para_M[0].PID_Vel_.P=0.15;  //0.1
			//FOC_Para_M[0].PID_current_q_.P=0.38;  //航模电机，速度闭环，不能大于1，否则容易失控
			//FOC_Para_M[0].PID_current_d_.P=0.38;  //0.5
			
			FOC_Para_M[0].Motor_State_.torque_controller = Type_foc_current;	
			//(0)Type_dc_current;  	(1)Type_foc_current;  (2)Type_voltage;
			FOC_Para_M[0].Motor_State_.controller = Type_velocity;				
			//(0)Type_torque;  			(1)Type_velocity;  		(2)Type_angle;
			
			FOC_Init_Flag[0] = 1;
			
		
			if(FOC_Init_Flag[0])
			{
				printf("Motor[0] ready.\r\n");
				vTaskDelete(Closedloop_FOC_0_Init_Task_Handle);
				break;
			}
		
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
  
}


/**********************************************************************
  * @ 函数名  ： Closedloop_FOC_1_Init_Task
  * @ 功能说明： Closedloop_FOC_1_Init_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Closedloop_FOC_1_Init_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(4); // 1000ms = 1s
//	unsigned int count_i=0;
//	float Vel_;
	
	#if M1_AS5600
		/* I2C3初始化	*/
//		MX_I2C3_Init();
		printf("AS5600\r\n");
	#elif M1_TLE5012B
		SPI2_Init_();              //TLE5012B
		printf("TLE5012B\r\n");
	#endif
	//vTaskDelay(100);
	
	

	
  while (1)
  {
			FOC_Para_M[1].Motor_State_.target = 6.28;
			MagneticSensor_Init(1,&hi2c2);     //AS5600 or TLE5012B
			InlineCurrentSense_Init(1,0.01, 50,4,5,NOT_SET,&hadc1);
			Motor_init(1,&htim3);
			Motor_initFOC(1,0,UNKNOWN);
			
//			FOC_Para_M[1].PID_Vel_.P=0.85;  //0.1
//			FOC_Para_M[1].PID_current_q_.P=0.5;  //航模电机，速度闭环，不能大于1，否则容易失控
//			FOC_Para_M[1].PID_current_d_.P=0.5;  //0.5
			
			FOC_Para_M[1].Motor_State_.torque_controller = Type_foc_current;	
			//(0)Type_dc_current;  	(1)Type_foc_current;  (2)Type_voltage;
			FOC_Para_M[1].Motor_State_.controller = Type_velocity;				
			//(0)Type_torque;  			(1)Type_velocity;  		(2)Type_angle;
			
			FOC_Init_Flag[1] = 1;
			
		
			if(FOC_Init_Flag[1])
			{
				printf("Motor[1] ready.\r\n");
				vTaskDelete(Closedloop_FOC_1_Init_Task_Handle);
				break;
			}
		
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
  
}


/**********************************************************************
  * @ 函数名  ： Closedloop_FOC_0_Task
  * @ 功能说明： Closedloop_FOC_0_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Closedloop_FOC_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(2); // 1000ms = 1s
	static uint32_t i = 0;
  while (1)
  {
			i = !i;
			if(FOC_Init_Flag[0]&&i)
			{
				move(0,FOC_Para_M[0].Motor_State_.target);
			
				loopFOC(0);
			
			}
			
			if(FOC_Init_Flag[1]&&!i)
			{
				move(1,FOC_Para_M[1].Motor_State_.target);
			
				loopFOC(1);
			
			}
			
			vTaskDelayUntil(&xLastWakeTime,xFrequency);
		}
  
}


/**********************************************************************
  * @ 函数名  ： USART1_Task
  * @ 功能说明： USART1_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void USART1_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(200);
	
	
	HAL_UART_Receive_IT(&huart1,&dr,1);
  while (1)
  {
//	printf("AccX_raw:%d\r\n AccY_raw:%d\r\n AccZ_raw:%d\r\n",MPU6050_raw_t.AccX,MPU6050_raw_t.AccY,MPU6050_raw_t.AccZ);
//	printf("GyroX_raw:%d\r\n GyroY_raw:%d\r\n GyroZ_raw:%d\r\n",MPU6050_raw_t.GyroX,MPU6050_raw_t.GyroY,MPU6050_raw_t.GyroZ);
//		printf("roll:%.3f\r\n pitch:%.3f\r\n yaw:%.3f\r\n",MPU6050_value.roll,MPU6050_value.pitch,MPU6050_value.yaw);
//		printf("MPU6050 temperature:%.3f\r\n",MPU6050_raw_t.Temp);
		
//		if(FOC_Init_Flag[0] == 1)
//		{
//			printf("---------------  FOC_Para_M[0]  ----------------------\r\n");
//			printf("-------------------------------------------------\r\n");
//			printf("TIM7_TICK = %d\r\n",TIM7_GetTick());
//			printf("Current0 angle = %f\r\n",FOC_Para_M[0].Motor_State_.shaft_angle);
//			printf("Current0 velocity = %f\r\n",FOC_Para_M[0].Motor_State_.shaft_velocity);
//			printf("Current0_a = %f\r\n",FOC_Para_M[0].Inline_Current_Param_.current.a);
//			printf("Current0_b = %f\r\n",FOC_Para_M[0].Inline_Current_Param_.current.b);
//			printf("Current0_c = %f\r\n",FOC_Para_M[0].Inline_Current_Param_.current.c);
//			printf("-------------------------------------------------\r\n");
//		}
//		
//		if(FOC_Init_Flag[1] == 1)
//		{
//			printf("---------------  FOC_Para_M[1]  ----------------------\r\n");
//			printf("-------------------------------------------------\r\n");
//			printf("TIM7_TICK = %d\r\n",TIM7_GetTick());
//			printf("Current1 angle = %f\r\n",FOC_Para_M[1].Motor_State_.shaft_angle);
//			printf("Current1 velocity = %f\r\n",FOC_Para_M[1].Motor_State_.shaft_velocity);
//			printf("Current1_a = %f\r\n",FOC_Para_M[1].Inline_Current_Param_.current.a);
//			printf("Current1_b = %f\r\n",FOC_Para_M[1].Inline_Current_Param_.current.b);
//			printf("Current1_c = %f\r\n",FOC_Para_M[1].Inline_Current_Param_.current.c);
//			printf("-------------------------------------------------\r\n");
//		}
		
//		if((USART_RX_STA&0x8000)!=0)
//				{
//					if(strncmp((char *)USART_RX_BUF, "0H", 2) == 0)
//					{
//						printf("Hello World!\r\n");
//					}
//					else if(strncmp((char *)USART_RX_BUF, "0T", 2) == 0)
//					{
//						FOC_Para_M[0].Motor_State_.target = atof((const char *)(USART_RX_BUF+2));
//						printf("RX=%.4f\r\n", FOC_Para_M[0].Motor_State_.target);
//					}
//					else if(strncmp((char *)USART_RX_BUF, "0P", 2) == 0)
//					{
//						FOC_Para_M[0].PID_Vel_.P = atof((const char *)(USART_RX_BUF+2));
//						printf("RX=%.4f\r\n", FOC_Para_M[0].PID_Vel_.P);
//					}
//					else if(strncmp((char *)USART_RX_BUF, "0I", 2) == 0)
//					{
//						FOC_Para_M[0].PID_Vel_.I = atof((const char *)(USART_RX_BUF+2));
//						printf("RX=%.4f\r\n", FOC_Para_M[0].PID_Vel_.I);
//					}
//					else if(strncmp((char *)USART_RX_BUF, "0D", 2) == 0)
//					{
//						FOC_Para_M[0].PID_Vel_.D = atof((const char *)(USART_RX_BUF+2));
//						printf("RX=%.4f\r\n", FOC_Para_M[0].PID_Vel_.D);
//					}
//					else if(strncmp((char *)USART_RX_BUF, "0O", 2) == 0)
//					{
//						FOC0_ON;
//						printf("ON OK!\r\n");
//					}
//					else if(strncmp((char *)USART_RX_BUF, "0F", 2) == 0)
//					{
//						FOC0_OFF;
//						printf("OFF OK!\r\n");
//					}
////					switch(USART_RX_BUF[0])
////					{
////						case 'H':
////							printf("Hello World!\r\n");
////							break;
////						case 'T':   //T6.28
////							FOC_Para_M[0].Motor_State_.target = atof((const char *)(USART_RX_BUF+1));
////							printf("RX=%.4f\r\n", FOC_Para_M[0].Motor_State_.target);
////							break;
////						case 'D':   //D
////							
////							printf("OFF OK!\r\n");
////							break;
////						case 'E':   //E
////							
////							printf("ON OK!\r\n");
////							break;
////					}
//					USART_RX_STA=0;
//				}
		//printf("-------------------------------------------------\r\n");
		vTaskDelay(xFrequency);
	}
			
}

/**********************************************************************
  * @ 函数名  ： USART3_Task
  * @ 功能说明： USART3_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
uint8_t SEND_NUM = 4;
static void USART3_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(60);
	
	float fdata[SEND_NUM];
	uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
	
	
//使用VOFA+上位机进行曲线绘制，便于PID调参数
  while (1)
  {
//		fdata[0] = MPU6050_value.roll - MPU6050_value_init.roll;
//		fdata[1] = MPU6050_value.pitch - MPU6050_value_init.pitch;
//		fdata[2] = MPU6050_value.yaw - MPU6050_value_init.yaw;
//		fdata[3] = MPU6050_raw_t.AccX;
//		fdata[4] = MPU6050_raw_t.AccY;
//		fdata[5] = MPU6050_raw_t.AccZ;
//		fdata[6] = MPU6050_raw_t.GyroX;
//		fdata[7] = MPU6050_raw_t.GyroY;
//		fdata[8] = MPU6050_raw_t.GyroZ;
//		fdata[9] = - MPU6050_value_init.roll;
//		fdata[10] =	- MPU6050_value_init.pitch;
//		fdata[11] = - MPU6050_value_init.yaw;
		
		fdata[0] = FOC_Para_M[0].Motor_State_.target;
		fdata[1] = FOC_Para_M[0].Motor_State_.shaft_velocity;
		fdata[2] = FOC_Para_M[1].Motor_State_.target;
		fdata[3] = FOC_Para_M[1].Motor_State_.shaft_velocity;

		
		HAL_UART_Transmit(&huart3,(uint8_t *)fdata,sizeof(float)*SEND_NUM,100);
		
		HAL_UART_Transmit(&huart3,tail,4,100);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
	/* ADC1初始化	*/
  MX_ADC1_Init();
	/* I2C2初始化	*/
  MX_I2C2_Init();
	/* I2C3初始化	*/
  MX_I2C3_Init();
	/* TIM3初始化	*/
  MX_TIM3_Init();
	/* TIM4初始化	*/
  MX_TIM4_Init();
	
	MX_TIM7_Init();
	/* USART1初始化	*/
  MX_USART1_UART_Init();
	/* USART2初始化	*/
  MX_USART2_UART_Init();
	/* USART3初始化	*/
  MX_USART3_UART_Init();
  
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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM7) {
    TIM7_IncTick();
		static uint32_t i = 0;
		uint32_t tick_temp;
		tick_temp = TIM7_GetTick();
		if(tick_temp%50 == 0)
		{
			i = !i;
			if(FOC_Init_Flag[0]&&i)
			{
				//loopFOC(0);
				//move(0,FOC_Para_M[0].Motor_State_.target);
			}
			if(FOC_Init_Flag[1]&&!i)
			{
				//loopFOC(1);
				//move(1,FOC_Para_M[1].Motor_State_.target);
			}
		}
  }
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
