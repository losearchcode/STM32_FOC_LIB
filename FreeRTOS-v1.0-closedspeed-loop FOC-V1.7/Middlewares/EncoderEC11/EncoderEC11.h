//---->>>>----文件描述：EC11旋转编码器底层驱动程序---<<<<----//
//---->>>>----文件版本：V1.0----<<<<----//
#ifndef __EncoderEC11_H
#define __EncoderEC11_H

#include	"main.h"

//----------------IO口定义----------------//

#define EC11_KEY_PIN                GPIO_PIN_13               
#define EC11_KEY_GPIO_PORT          GPIOB                    
#define EC11_KEY_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define EC11_A_PIN                  GPIO_PIN_12               
#define EC11_A_GPIO_PORT            GPIOB                    
#define EC11_A_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

#define EC11_B_PIN                  GPIO_PIN_14               
#define EC11_B_GPIO_PORT            GPIOB                    
#define EC11_B_CLK_ENABLE()    			__GPIOB_CLK_ENABLE()

#define EC11_A_Now                  HAL_GPIO_ReadPin(EC11_A_GPIO_PORT,EC11_A_PIN)//EC11的A引脚，视为时钟线
#define EC11_B_Now                  HAL_GPIO_ReadPin(EC11_B_GPIO_PORT,EC11_B_PIN)//EC11的B引脚，视为信号线
#define EC11_Key                    HAL_GPIO_ReadPin(EC11_KEY_GPIO_PORT,EC11_KEY_PIN)//EC11的按键

//----------------编码器动作代码相关定义----------------//

static unsigned char EC11_NUM_SW = 0;

//----------------用户自定义变量----------------//
typedef struct {
		uint8_t EC11_SCAN_PERIOD_MS_t;
		uint8_t EC11_Analyze_Value;
		uint8_t EC11_Analyze_Last_Value;
} EncoderEC11_t;

extern EncoderEC11_t EC11;

extern uint8_t EC11_Init_Flag;

//----------------编码器参数微调宏定义----------------//
#define EC11_SCAN_PERIOD_MS         EC11.EC11_SCAN_PERIOD_MS_t                     //EC11编码器扫描周期
#define KEY_COUNT_DESHAKING         ( 20/EC11_SCAN_PERIOD_MS)       //按键消抖时间
#define KEY_COUNT_LONGTIME          (600/EC11_SCAN_PERIOD_MS)       //长按按键判断时间
#define KEY_COUNT_DUALCLICKTIME     (150/EC11_SCAN_PERIOD_MS)       //双击按键判断时间
#define KEY_LONG_REPEAT_TIME        (200/EC11_SCAN_PERIOD_MS)       //长按按键的回报率的倒数，即一直长按按键时响应的时间间隔

//----------------局部文件内变量列表----------------//
static  char    Click_turn_flag=0;
static  char    EC11_A_Last = 0;                        //EC11的A引脚上一次的状态
static  char    EC11_B_Last = 0;                        //EC11的B引脚上一次的状态
static  char    EC11_Type = 1;                          //定义变量暂存EC11的类型---->>>>----  0：一定位对应一脉冲；  1：两定位对应一脉冲
                                                        //所谓一定位对应一脉冲，是指EC11旋转编码器每转动一格，A和B都会输出一个完整的方波。
                                                        //而  两定位对应一脉冲，是指EC11旋转编码器每转动两格，A和B才会输出一个完整的方波，只转动一格只输出A和B的上升沿或下降沿
                                                        
static   int    EC11_KEY_COUNT = 0;                     //EC11按键动作计数器
static   int    EC11_KEY_DoubleClick_Count = 0;         //EC11按键双击动作计数器
static  char    FLAG_EC11_KEY_ShotClick = 0;            //EC11按键短按动作标志
static  char    FLAG_EC11_KEY_LongClick = 0;            //EC11按键长按动作标志
static  char    FLAG_EC11_KEY_DoubleClick = 0;          //EC11按键双击动作标志



//----------------函数快速调用（复制粘贴）列表----------------//
//
/*******************************************************************
void Encoder_EC11_Init(unsigned char Set_EC11_TYPE);        //初始化EC11旋转编码器IO口和类型以及变量初始化
char Encoder_EC11_Scan();                                   //扫描旋转编码器的动作
void Encoder_EC11_Analyze(char EC11_Value);                 //分析EC11旋转编码器的动作以及动作处理代码
******************************************************************/
//-------->>>>>>>>--------注意事项：EC11旋转编码器的扫描时间间隔控制在1~4ms之间，否则5ms及以上的扫描时间在快速旋转时可能会误判旋转方向--------<<<<<<<<--------//
//-------->>>>>>>>--------注意事项：EC11旋转编码器的扫描时间间隔控制在1~4ms之间，否则5ms及以上的扫描时间在快速旋转时可能会误判旋转方向--------<<<<<<<<--------//
//-------->>>>>>>>--------注意事项：EC11旋转编码器的扫描时间间隔控制在1~4ms之间，否则5ms及以上的扫描时间在快速旋转时可能会误判旋转方向--------<<<<<<<<--------//

//----------------函数声明列表----------------//
//
//*******************************************************************/
//功能：初始化EC11旋转编码器相关参数
//形参：EC11旋转编码器的类型-->>  unsigned char Set_EC11_TYPE  <<--  ：0----一定位对应一脉冲；1（或非0）----两定位对应一脉冲。
//返回：无
//详解：对EC11旋转编码器的连接IO口做IO口模式设置。以及将相关的变量进行初始化
//*******************************************************************/
void Encoder_EC11_Init(unsigned char Set_EC11_TYPE,uint8_t ec11_scan_ms);

//*******************************************************************/
//功能：扫描EC11旋转编码器的动作并将参数返回给动作分析函数使用
//形参：EC11旋转编码器的类型-->>  unsigned char Set_EC11_TYPE  <<--  ：0----一定位对应一脉冲；1（或非0）----两定位对应一脉冲
//返回：EC11旋转编码器的扫描结果-->>  char ScanResult  -->>  0：无动作；1：正转； -1：反转；2：只按下按键；3：按着按键正转；-3：按着按键反转
//详解：只扫描EC11旋转编码器有没有动作，不关心是第几次按下按键或长按或双击。返回值直接作为形参传给 [ void Encoder_EC11_Analyze(char EC11_Value); ] 函数使用
//*******************************************************************/
int Encoder_EC11_Scan(void);
    
//*******************************************************************/
//功能：对EC11旋转编码器的动作进行分析，并作出相应的动作处理代码
//形参：无
//返回：char AnalyzeResult = 0;目前无用。若在该函数里做了动作处理，则函数的返回值无需理会
//详解：对EC11旋转编码器的动作进行模式分析，是单击还是双击还是长按松手还是一直按下。形参从 [ char Encoder_EC11_Scan(unsigned char Set_EC11_TYPE) ] 函数传入。在本函数内修改需要的动作处理代码
//*******************************************************************/
uint8_t Encoder_EC11_Analyze(int EC11_Value);

#endif


//---->>>>----函数使用示例----<<<<----//
/********

#include "config.h"
#include "delay.h"
#include "EncoderEC11.h"


int cnt = -1;                                                                   //流水灯速查表偏移变量
unsigned char disp_tmp[] = {~0x01,~0x02,~0x04,~0x08,~0x10,~0x20,~0x40,~0x80};   //流水灯速查表

void Timer0Init(void)       //1毫秒@22.1184MHz
{
    AUXR &= 0x7F;       //定时器时钟12T模式
    TMOD &= 0xF0;       //设置定时器模式
    TL0 = 0xCD;     //设置定时初值
    TH0 = 0xF8;     //设置定时初值
    TF0 = 0;        //清除TF0标志
    TR0 = 1;        //定时器0开始计时
}

void main()
{

    P1_QB_ALL();
    P2_QB_ALL();
    P3_QB_ALL();
    delay_ms(50);  //延时100毫秒等待所有MCU复位

    Encoder_EC11_Init(1);

    EA = 1;
    ET0 = 1;
    Timer0Init();
    while(1)
    {
            
    }
 
}

//-------->>>>>>>>--------注意事项：EC11旋转编码器的扫描时间间隔控制在1~4ms之间，否则5ms及以上的扫描时间在快速旋转时可能会误判旋转方向--------<<<<<<<<--------//
void T0_ISR() interrupt 1
{
    static int tmp =0;
    Encoder_EC11_Analyze(Encoder_EC11_Scan());
    if(P33 == 0)
    {
        tmp ++;
        if(tmp == 500)
        {
            tmp =0;
            P27 = !P27;
        }
    }   
}


********/
