#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"

#define USE_HORIZONTAL 0  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏


#define LCD_W 240
#define LCD_H 240

#define SCLK_PIN                  GPIO_PIN_9                 
#define SCLK_GPIO_PORT            GPIOB                     
#define SCLK_GPIO_CLK             __GPIOB_CLK_ENABLE

#define SDIN_PIN                  GPIO_PIN_8                 
#define SDIN_GPIO_PORT            GPIOB                    
#define SDIN_GPIO_CLK             __GPIOB_CLK_ENABLE()

#define RST_PIN                  	GPIO_PIN_7                
#define RST_GPIO_PORT            	GPIOB                 
#define RST_GPIO_CLK             	__GPIOB_CLK_ENABLE()

#define DC_PIN                  	GPIO_PIN_6                 
#define DC_GPIO_PORT            	GPIOB                      
#define DC_GPIO_CLK             	__GPIOB_CLK_ENABLE()

#define BLK_PIN                  	GPIO_PIN_5                 
#define BLK_GPIO_PORT            	GPIOB                      
#define BLK_GPIO_CLK             	__GPIOB_CLK_ENABLE()

//#define VCC_PIN                  	GPIO_Pin_15                 
//#define VCC_GPIO_PORT            	GPIOB                      
//#define VCC_GPIO_CLK             	RCC_APB2Periph_GPIOB

//#define GND_PIN                  	GPIO_Pin_9                 
//#define GND_GPIO_PORT            	GPIOD                      
//#define GND_GPIO_CLK             	RCC_AHB1Periph_GPIOD

//-----------------LCD端口定义---------------- 

#define LCD_SCLK_Clr() HAL_GPIO_WritePin(SCLK_GPIO_PORT,SCLK_PIN,GPIO_PIN_RESET)//SCL=SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(SCLK_GPIO_PORT,SCLK_PIN,GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(SDIN_GPIO_PORT,SDIN_PIN,GPIO_PIN_RESET)//SDA=MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(SDIN_GPIO_PORT,SDIN_PIN,GPIO_PIN_SET)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(RST_GPIO_PORT,RST_PIN,GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(RST_GPIO_PORT,RST_PIN,GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(DC_GPIO_PORT,DC_PIN,GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(DC_GPIO_PORT,DC_PIN,GPIO_PIN_SET)
 		     
//#define LCD_CS_Clr()   GPIO_ResetBits(GPIOD,GPIO_Pin_1)//CS
//#define LCD_CS_Set()   GPIO_SetBits(GPIOD,GPIO_Pin_1)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(BLK_GPIO_PORT,BLK_PIN,GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(BLK_GPIO_PORT,BLK_PIN,GPIO_PIN_SET)


extern uint8_t LCD_init_flag;

void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(uint8_t dat);//模拟SPI时序
void LCD_WR_DATA8(uint8_t dat);//写入一个字节
void LCD_WR_DATA(uint16_t dat);//写入两个字节
void LCD_WR_REG(uint8_t dat);//写入一个指令
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




