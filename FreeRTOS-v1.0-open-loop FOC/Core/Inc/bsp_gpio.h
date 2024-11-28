#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"

//���Ŷ���
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


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  GPIO_PIN_SET
#define OFF GPIO_PIN_RESET

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN,~a)
#define FOC_EN(a)	HAL_GPIO_WritePin(FOC_EN_GPIO_PORT,FOC_EN_PIN,a)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			  //����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BSRR=(uint32_t)i << 16;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬


/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON			digitalLo(LED1_GPIO_PORT,LED1_PIN)

void MX_GPIO_Init(void);


#endif

