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

static TaskHandle_t MPU6050_Task_Handle = NULL;/* KEY������ */

static TaskHandle_t USART1_Task_Handle = NULL;

static TaskHandle_t USART3_Task_Handle = NULL;

static TaskHandle_t Closedloop_FOC_0_Init_Task_Handle = NULL;

static TaskHandle_t Closedloop_FOC_1_Init_Task_Handle = NULL;

static TaskHandle_t Closedloop_FOC_Task_Handle = NULL;


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
 **********************************************************************************************/

/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */

static void MPU6050_Task(void* parameter);

static void USART1_Task(void* parameter);

static void USART3_Task(void* parameter);

static void Closedloop_FOC_0_Init_Task(void* parameter);

static void Closedloop_FOC_1_Init_Task(void* parameter);

static void Closedloop_FOC_Task(void* parameter);


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
//  xReturn = xTaskCreate((TaskFunction_t )MPU6050_Task,  /* ������ں��� */
//                        (const char*    )"MPU6050_Task",/* �������� */
//                        (uint16_t       )256,  /* ����ջ��С */
//                        (void*          )NULL,/* ������ں������� */
//                        (UBaseType_t    )4, /* ��������ȼ� */
//                        (TaskHandle_t*  )&MPU6050_Task_Handle);/* ������ƿ�ָ�� */ 
//  if(pdPASS == xReturn)
//    printf("���� LED_Task ����ɹ�!\n\n");
	

		  /* ����LED_Task���� */
  xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_0_Init_Task,  /* ������ں��� */
                        (const char*    )"Closedloop_FOC_0_Init_Task",/* �������� */
                        (uint16_t       )256,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )6, /* ��������ȼ� */
                        (TaskHandle_t*  )&Closedloop_FOC_0_Init_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� Closedloop_FOC_0_Init_Task ����ɹ�!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_1_Init_Task,  /* ������ں��� */
							(const char*    )"Closedloop_FOC_1_Init_Task",/* �������� */
							(uint16_t       )256,  /* ����ջ��С */
							(void*          )NULL,/* ������ں������� */
							(UBaseType_t    )6, /* ��������ȼ� */
							(TaskHandle_t*  )&Closedloop_FOC_1_Init_Task_Handle);/* ������ƿ�ָ�� */ 
	if(pdPASS == xReturn)
		printf("���� Closedloop_FOC_1_Init_Task ����ɹ�!\n\n");
	
	xReturn = xTaskCreate((TaskFunction_t )Closedloop_FOC_Task,  /* ������ں��� */
							(const char*    )"Closedloop_FOC_0_Task",/* �������� */
							(uint16_t       )256,  /* ����ջ��С */
							(void*          )NULL,/* ������ں������� */
							(UBaseType_t    )5, /* ��������ȼ� */
							(TaskHandle_t*  )&Closedloop_FOC_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� Closedloop_FOC_0_Task ����ɹ�!\n\n");
	

	
	xReturn = xTaskCreate((TaskFunction_t )USART1_Task,  /* ������ں��� */
                        (const char*    )"USART1_Task",/* �������� */
                        (uint16_t       )256,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )4, /* ��������ȼ� */
                        (TaskHandle_t*  )&USART1_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� USART1_Task ����ɹ�!\n\n");	
	
	xReturn = xTaskCreate((TaskFunction_t )USART3_Task,  /* ������ں��� */
                        (const char*    )"USART3_Task",/* �������� */
                        (uint16_t       )256,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )4, /* ��������ȼ� */
                        (TaskHandle_t*  )&USART3_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("���� USART3_Task ����ɹ�!\n\n");	
	
	
												
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}

/**********************************************************************
  * @ ������  �� LED_Task
  * @ ����˵���� LED_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
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
  * @ ������  �� Closedloop_FOC_0_Init_Task
  * @ ����˵���� Closedloop_FOC_0_Init_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
uint8_t FOC_Init_Flag[2] = {0,0};
static void Closedloop_FOC_0_Init_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(4); // 1000ms = 1s
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
	//vTaskDelay(100);
	
  while (1)
  {
			FOC_Para_M[0].Motor_State_.target = 6.28;
			MagneticSensor_Init(0,&hi2c3);     //AS5600 or TLE5012B
			InlineCurrentSense_Init(0,0.01, 50,6,7,NOT_SET,&hadc1);
			Motor_init(0,&htim4);
			Motor_initFOC(0,0,UNKNOWN);
			
			FOC_Para_M[0].PID_Vel_.P=0.15;  //0.1
			//FOC_Para_M[0].PID_current_q_.P=0.38;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
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
  * @ ������  �� Closedloop_FOC_1_Init_Task
  * @ ����˵���� Closedloop_FOC_1_Init_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void Closedloop_FOC_1_Init_Task(void* parameter)
{	
	TickType_t xLastWakeTime;
	
	const TickType_t xFrequency = pdMS_TO_TICKS(4); // 1000ms = 1s
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
	//vTaskDelay(100);
	
	

	
  while (1)
  {
			FOC_Para_M[1].Motor_State_.target = 6.28;
			MagneticSensor_Init(1,&hi2c2);     //AS5600 or TLE5012B
			InlineCurrentSense_Init(1,0.01, 50,4,5,NOT_SET,&hadc1);
			Motor_init(1,&htim3);
			Motor_initFOC(1,0,UNKNOWN);
			
//			FOC_Para_M[1].PID_Vel_.P=0.85;  //0.1
//			FOC_Para_M[1].PID_current_q_.P=0.5;  //��ģ������ٶȱջ������ܴ���1����������ʧ��
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
  * @ ������  �� Closedloop_FOC_0_Task
  * @ ����˵���� Closedloop_FOC_0_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
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
  * @ ������  �� USART1_Task
  * @ ����˵���� USART1_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
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
  * @ ������  �� USART3_Task
  * @ ����˵���� USART3_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
uint8_t SEND_NUM = 4;
static void USART3_Task(void* parameter)
{	
	
	const TickType_t xFrequency = pdMS_TO_TICKS(60);
	
	float fdata[SEND_NUM];
	uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
	
	
//ʹ��VOFA+��λ���������߻��ƣ�����PID������
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
	/* ADC1��ʼ��	*/
  MX_ADC1_Init();
	/* I2C2��ʼ��	*/
  MX_I2C2_Init();
	/* I2C3��ʼ��	*/
  MX_I2C3_Init();
	/* TIM3��ʼ��	*/
  MX_TIM3_Init();
	/* TIM4��ʼ��	*/
  MX_TIM4_Init();
	
	MX_TIM7_Init();
	/* USART1��ʼ��	*/
  MX_USART1_UART_Init();
	/* USART2��ʼ��	*/
  MX_USART2_UART_Init();
	/* USART3��ʼ��	*/
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
