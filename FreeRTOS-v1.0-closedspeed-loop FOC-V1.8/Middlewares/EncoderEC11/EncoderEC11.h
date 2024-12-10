//---->>>>----�ļ�������EC11��ת�������ײ���������---<<<<----//
//---->>>>----�ļ��汾��V1.0----<<<<----//
#ifndef __EncoderEC11_H
#define __EncoderEC11_H

#include	"main.h"

//----------------IO�ڶ���----------------//

#define EC11_KEY_PIN                GPIO_PIN_13               
#define EC11_KEY_GPIO_PORT          GPIOB                    
#define EC11_KEY_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define EC11_A_PIN                  GPIO_PIN_12               
#define EC11_A_GPIO_PORT            GPIOB                    
#define EC11_A_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

#define EC11_B_PIN                  GPIO_PIN_14               
#define EC11_B_GPIO_PORT            GPIOB                    
#define EC11_B_CLK_ENABLE()    			__GPIOB_CLK_ENABLE()

#define EC11_A_Now                  HAL_GPIO_ReadPin(EC11_A_GPIO_PORT,EC11_A_PIN)//EC11��A���ţ���Ϊʱ����
#define EC11_B_Now                  HAL_GPIO_ReadPin(EC11_B_GPIO_PORT,EC11_B_PIN)//EC11��B���ţ���Ϊ�ź���
#define EC11_Key                    HAL_GPIO_ReadPin(EC11_KEY_GPIO_PORT,EC11_KEY_PIN)//EC11�İ���

//----------------����������������ض���----------------//

static unsigned char EC11_NUM_SW = 0;

//----------------�û��Զ������----------------//
typedef struct {
		uint8_t EC11_SCAN_PERIOD_MS_t;
		uint8_t EC11_Analyze_Value;
		uint8_t EC11_Analyze_Last_Value;
} EncoderEC11_t;

extern EncoderEC11_t EC11;

extern uint8_t EC11_Init_Flag;

//----------------����������΢���궨��----------------//
#define EC11_SCAN_PERIOD_MS         EC11.EC11_SCAN_PERIOD_MS_t                     //EC11������ɨ������
#define KEY_COUNT_DESHAKING         ( 20/EC11_SCAN_PERIOD_MS)       //��������ʱ��
#define KEY_COUNT_LONGTIME          (600/EC11_SCAN_PERIOD_MS)       //���������ж�ʱ��
#define KEY_COUNT_DUALCLICKTIME     (150/EC11_SCAN_PERIOD_MS)       //˫�������ж�ʱ��
#define KEY_LONG_REPEAT_TIME        (200/EC11_SCAN_PERIOD_MS)       //���������Ļر��ʵĵ�������һֱ��������ʱ��Ӧ��ʱ����

//----------------�ֲ��ļ��ڱ����б�----------------//
static  char    Click_turn_flag=0;
static  char    EC11_A_Last = 0;                        //EC11��A������һ�ε�״̬
static  char    EC11_B_Last = 0;                        //EC11��B������һ�ε�״̬
static  char    EC11_Type = 1;                          //��������ݴ�EC11������---->>>>----  0��һ��λ��Ӧһ���壻  1������λ��Ӧһ����
                                                        //��νһ��λ��Ӧһ���壬��ָEC11��ת������ÿת��һ��A��B�������һ�������ķ�����
                                                        //��  ����λ��Ӧһ���壬��ָEC11��ת������ÿת������A��B�Ż����һ�������ķ�����ֻת��һ��ֻ���A��B�������ػ��½���
                                                        
static   int    EC11_KEY_COUNT = 0;                     //EC11��������������
static   int    EC11_KEY_DoubleClick_Count = 0;         //EC11����˫������������
static  char    FLAG_EC11_KEY_ShotClick = 0;            //EC11�����̰�������־
static  char    FLAG_EC11_KEY_LongClick = 0;            //EC11��������������־
static  char    FLAG_EC11_KEY_DoubleClick = 0;          //EC11����˫��������־



//----------------�������ٵ��ã�����ճ�����б�----------------//
//
/*******************************************************************
void Encoder_EC11_Init(unsigned char Set_EC11_TYPE);        //��ʼ��EC11��ת������IO�ں������Լ�������ʼ��
char Encoder_EC11_Scan();                                   //ɨ����ת�������Ķ���
void Encoder_EC11_Analyze(char EC11_Value);                 //����EC11��ת�������Ķ����Լ������������
******************************************************************/
//-------->>>>>>>>--------ע�����EC11��ת��������ɨ��ʱ����������1~4ms֮�䣬����5ms�����ϵ�ɨ��ʱ���ڿ�����תʱ���ܻ�������ת����--------<<<<<<<<--------//
//-------->>>>>>>>--------ע�����EC11��ת��������ɨ��ʱ����������1~4ms֮�䣬����5ms�����ϵ�ɨ��ʱ���ڿ�����תʱ���ܻ�������ת����--------<<<<<<<<--------//
//-------->>>>>>>>--------ע�����EC11��ת��������ɨ��ʱ����������1~4ms֮�䣬����5ms�����ϵ�ɨ��ʱ���ڿ�����תʱ���ܻ�������ת����--------<<<<<<<<--------//

//----------------���������б�----------------//
//
//*******************************************************************/
//���ܣ���ʼ��EC11��ת��������ز���
//�βΣ�EC11��ת������������-->>  unsigned char Set_EC11_TYPE  <<--  ��0----һ��λ��Ӧһ���壻1�����0��----����λ��Ӧһ���塣
//���أ���
//��⣺��EC11��ת������������IO����IO��ģʽ���á��Լ�����صı������г�ʼ��
//*******************************************************************/
void Encoder_EC11_Init(unsigned char Set_EC11_TYPE,uint8_t ec11_scan_ms);

//*******************************************************************/
//���ܣ�ɨ��EC11��ת�������Ķ��������������ظ�������������ʹ��
//�βΣ�EC11��ת������������-->>  unsigned char Set_EC11_TYPE  <<--  ��0----һ��λ��Ӧһ���壻1�����0��----����λ��Ӧһ����
//���أ�EC11��ת��������ɨ����-->>  char ScanResult  -->>  0���޶�����1����ת�� -1����ת��2��ֻ���°�����3�����Ű�����ת��-3�����Ű�����ת
//��⣺ֻɨ��EC11��ת��������û�ж������������ǵڼ��ΰ��°����򳤰���˫��������ֱֵ����Ϊ�βδ��� [ void Encoder_EC11_Analyze(char EC11_Value); ] ����ʹ��
//*******************************************************************/
int Encoder_EC11_Scan(void);
    
//*******************************************************************/
//���ܣ���EC11��ת�������Ķ������з�������������Ӧ�Ķ����������
//�βΣ���
//���أ�char AnalyzeResult = 0;Ŀǰ���á����ڸú��������˶������������ķ���ֵ�������
//��⣺��EC11��ת�������Ķ�������ģʽ�������ǵ�������˫�����ǳ������ֻ���һֱ���¡��βδ� [ char Encoder_EC11_Scan(unsigned char Set_EC11_TYPE) ] �������롣�ڱ��������޸���Ҫ�Ķ����������
//*******************************************************************/
uint8_t Encoder_EC11_Analyze(int EC11_Value);

#endif


//---->>>>----����ʹ��ʾ��----<<<<----//
/********

#include "config.h"
#include "delay.h"
#include "EncoderEC11.h"


int cnt = -1;                                                                   //��ˮ���ٲ��ƫ�Ʊ���
unsigned char disp_tmp[] = {~0x01,~0x02,~0x04,~0x08,~0x10,~0x20,~0x40,~0x80};   //��ˮ���ٲ��

void Timer0Init(void)       //1����@22.1184MHz
{
    AUXR &= 0x7F;       //��ʱ��ʱ��12Tģʽ
    TMOD &= 0xF0;       //���ö�ʱ��ģʽ
    TL0 = 0xCD;     //���ö�ʱ��ֵ
    TH0 = 0xF8;     //���ö�ʱ��ֵ
    TF0 = 0;        //���TF0��־
    TR0 = 1;        //��ʱ��0��ʼ��ʱ
}

void main()
{

    P1_QB_ALL();
    P2_QB_ALL();
    P3_QB_ALL();
    delay_ms(50);  //��ʱ100����ȴ�����MCU��λ

    Encoder_EC11_Init(1);

    EA = 1;
    ET0 = 1;
    Timer0Init();
    while(1)
    {
            
    }
 
}

//-------->>>>>>>>--------ע�����EC11��ת��������ɨ��ʱ����������1~4ms֮�䣬����5ms�����ϵ�ɨ��ʱ���ڿ�����תʱ���ܻ�������ת����--------<<<<<<<<--------//
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
