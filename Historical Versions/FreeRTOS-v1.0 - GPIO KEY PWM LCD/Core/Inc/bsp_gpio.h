#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "main.h"

//引脚定义
/*******************************************************/

#define LED1_PIN                  GPIO_PIN_13               
#define LED1_GPIO_PORT            GPIOC                     
#define LED1_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()

#define KEY1_PIN                  GPIO_PIN_0               
#define KEY1_GPIO_PORT            GPIOA                    
#define KEY1_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  GPIO_PIN_RESET
#define OFF GPIO_PIN_SET

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN,a)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			  //设置为高电平		
#define digitalLo(p,i)			{p->BSRR=(uint32_t)i << 16;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态


/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF		digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON			digitalLo(LED1_GPIO_PORT,LED1_PIN)

void MX_GPIO_Init(void);


#endif

