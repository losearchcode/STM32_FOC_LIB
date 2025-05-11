#include "bsp_mpu_6050.h"
#include "cmsis_os.h"

I2C_BUS MPU6050_I2C;
MPU6050_raw MPU6050_raw_t;
MPU6050 MPU6050_value,MPU6050_value_init;
float MPU6050_temperature;

typedef enum{
    Band_256Hz = 0x00,
    Band_186Hz,
    Band_96Hz,
    Band_43Hz,
    Band_21Hz,
    Band_10Hz,
    Band_5Hz
}Filter_Typedef;

typedef enum{
    gyro_250 = 0x00,
    gyro_500 = 0x08,
    gyro_1000 = 0x10,
    gyro_2000 = 0x18
}GYRO_CONFIG_Typedef;

typedef enum{
    acc_2g = 0x00,
    acc_4g = 0x08,
    acc_8g = 0x10,
    acc_16g = 0x18
}ACCEL_CONFIG_Typedef;

typedef enum{
    FIFO_Disable,
    Acc_OUT = 0x08,
    Gyro_zOUT = 0x10,
    Gyro_yOUT = 0x20,
    Gyro_xOUT = 0x40,
    Temp_OUT =0x80,
}FIFO_EN_Typedef;

typedef enum{
    interrupt_Disable,
    Data_Ready_EN = 0x01,
    I2C_Master_EN = 0x08,
    FIFO_overFolow_EN = 0x10,
    Motion_EN = 0x40,
}INT_EN_Typedef;
///////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct MPU6050_InitTypeDef
{
    uint16_t SMPLRT_Rate;						//输出频率开了低通滤波 最高1khz 加速度计最高1khz
    Filter_Typedef Filter;					
    GYRO_CONFIG_Typedef gyro_range;
    ACCEL_CONFIG_Typedef acc_range;
    FIFO_EN_Typedef FIFO_EN;
    INT_EN_Typedef INT;
}MPU6050_InitTypeDef;
///////////////////////////////////////////////////////////////////////////////////////////////////
static void MPU6050_Register_init(MPU6050_InitTypeDef* this){
    
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_PWR_MGMT_1,0x80);//复位
	
    vTaskDelay(100);
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_PWR_MGMT_1,0x00);//唤醒
	
   	uint8_t SMPLRT_DIV;
		if(this->SMPLRT_Rate>1000)this->SMPLRT_Rate=1000;
		else if(this->SMPLRT_Rate<4)this->SMPLRT_Rate=4;
		SMPLRT_DIV=1000/this->SMPLRT_Rate-1;//由计算公式得
	
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_SMPLRT_DIV,SMPLRT_DIV);

    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_INT_ENABLE,this->INT);
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_CONFIG,this->Filter);
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_GYRO_CONFIG,this->gyro_range);
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_ACCEL_CONFIG,this->acc_range);
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_FIFO_EN,this->FIFO_EN);
    uint8_t temp = 0x00;
    if(this->FIFO_EN!=0x00)//如果打开了FIFO
        temp = 0x40;
    if((this->INT & 0x08)==1)//如果打开了中断
        temp |= 0x08;
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_USER_CTRL,temp);
    I2C_P(MPU6050_I2C)->Write_Reg_Word(MPU6050_PWR_MGMT_1,0x01);//X轴为参考
}

void MPU6050_init(I2C_HandleTypeDef* I2Cx){
    MPU6050_I2C = Create_HI2C(I2Cx,MPU6050_ADDRESS);
    
    MPU6050_InitTypeDef MPU6050_init_Struct;
    MPU6050_init_Struct.SMPLRT_Rate = 200;            //采样率Hz
    MPU6050_init_Struct.Filter = Band_186Hz;          //滤波器带宽
    MPU6050_init_Struct.gyro_range = gyro_2000;       //陀螺仪测量范围
    MPU6050_init_Struct.acc_range = acc_16g;          //加速度计测量范围
    MPU6050_init_Struct.FIFO_EN = FIFO_Disable;       //FIFO
    MPU6050_init_Struct.INT = interrupt_Disable;      //中断配置
    
    MPU6050_Register_init(&MPU6050_init_Struct);      //初始化寄存器
    
}



/**
  * 函数功能：线性标度变换
  * 入口参数：Sample_Value: 采样回来的原始数值 
  * 入口参数：URV：         量程上限      
  * 入口参数：LRV：         量程下限
  * 返 回 值：变换后的数据
  */
float Scale_Transform(float Sample_Value, float URV, float LRV)
{
    float Data;             //定义用来保存变换后的数据变量
    float Value_L = -32767.0; //定义采样值下限变量   MPU6050寄存器是16位的，最高位是符号位，
    float Value_U = 32767.0;  //定义采样值上限变量   所以寄存器输出范围是-7FFF~7FFF,对应十进制-32767~32767
    
    /* 公式：当前数据 =（采样值 - 采样值下限）/（采样值上限 - 采样值下限）*（量程上限 - 量程下限）+ 量程下限     */
    Data = (Sample_Value - Value_L) / (Value_U - Value_L) * (URV - LRV) + LRV;
           
    return Data;
}



void MPU6050_Get_Raw(MPU6050_raw* this){
		uint8_t temp[2];
		
		
    int16_t GYRO_Array[3];     			//定义用来保存陀螺仪原始数值的数组
		int16_t ACCEL_Array[3];         //定义用来保存加速度计原始数值的变量
		int16_t tempre;
	
	
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_ACCEL_XOUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_ACCEL_XOUT_H,&temp[1],1);
	
    ACCEL_Array[0] = ((int16_t)((temp[1]<<8) | temp[0]));
	
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_ACCEL_YOUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_ACCEL_YOUT_H,&temp[1],1);
	
    ACCEL_Array[1] = ((int16_t)((temp[1]<<8) | temp[0]));
	
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_ACCEL_ZOUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_ACCEL_ZOUT_H,&temp[1],1);
	
    ACCEL_Array[2] = ((int16_t)((temp[1]<<8) | temp[0]));
	
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_GYRO_XOUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_GYRO_XOUT_H,&temp[1],1);
	
    GYRO_Array[0] = ((int16_t)((temp[1]<<8) | temp[0]));
	
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_GYRO_YOUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_GYRO_YOUT_H,&temp[1],1);
	
    GYRO_Array[1] = ((int16_t)((temp[1]<<8) | temp[0]));
		
		
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_GYRO_ZOUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_GYRO_ZOUT_H,&temp[1],1);
	
    GYRO_Array[2] = ((int16_t)((temp[1]<<8) | temp[0]));
		
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_TEMP_OUT_L,&temp[0],1);
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_TEMP_OUT_H,&temp[1],1);
	
    tempre = ((int16_t)((temp[1]<<8) | temp[0]));
		
		this->AccX = 1.088f * Scale_Transform( (float)ACCEL_Array[0], 16.0, -16.0);  //转换X轴
		this->AccY = 0.980f * Scale_Transform( (float)ACCEL_Array[1], 16.0, -16.0);  //转换X轴
		this->AccZ = 0.809f * Scale_Transform( (float)ACCEL_Array[2], 16.0, -16.0);  //转换X轴
		this->GyroX = Scale_Transform( (float)GYRO_Array[0], 2000.0, -2000.0) + 1.6025f;  //转换X轴
		this->GyroY = Scale_Transform( (float)GYRO_Array[1], 2000.0, -2000.0) - 2.487f;  //转换X轴
		this->GyroZ = Scale_Transform( (float)GYRO_Array[2], 2000.0, -2000.0) + 2.625f;  //转换X轴
		
		this->Temp = (float)tempre/340.0f + 36.53f;
		
	
}

float MPU6050_GetTemp(void){
	uint8_t temp[2];
	int16_t temper;
	
	I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_TEMP_OUT_L,&temp[0],1);
	I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_TEMP_OUT_H,&temp[1],1);

	temper = ((int16_t)((temp[1]<<8) | temp[0]));
	
	float temperature = (float)temper/340.0f + 36.53f;
	return temperature;
}

void MPU6050_Get_Angle(MPU6050* this , MPU6050_raw* MPU6050_raw_t){
	float Ax,Ay,Az=0.0f;
	float Gx,Gy,Gz=0.0f;
	static float Gyroscope_roll=0.0f;
	static float Gyroscope_pitch=0.0f;
  const static float dt=0.02f;
  const static float weight = 0.65f;//权重

    //static int roll_befor = 0;
    //static int pitch_befor = 0;
	
	MPU6050_Get_Raw(MPU6050_raw_t);
	
	Ax = MPU6050_raw_t->AccX ;


	Ay = MPU6050_raw_t->AccY ;


	Az = MPU6050_raw_t->AccZ ;


	Gx = MPU6050_raw_t->GyroX *dt;


	Gy = MPU6050_raw_t->GyroY *dt;


	Gz = MPU6050_raw_t->GyroZ *dt;

	Gyroscope_roll+=Gy;
	Gyroscope_pitch+=Gx;
	this->roll=weight * atan2(Ay,Az)/3.1415926f * 180.0f + (1.0f-weight) * Gyroscope_roll;
	this->pitch=-(weight * atan2(Ax,Az)/3.1415926f * 180.0f + (1.0f-weight) * Gyroscope_pitch);
	this->yaw += Gz + 0.000175f;//减小零飘

    //this->roll = (this->roll+roll_befor)/2;
    //this->pitch = (this->pitch+pitch_befor)/2;
    //roll_befor = this->roll;
    //pitch_befor = this->pitch;
}

uint8_t MPU6050_ID_Get(void){
		uint8_t temp;
		I2C_P(MPU6050_I2C)->Read_Reg_Cont(MPU6050_WHO_AM_I,&temp,1);
    return temp;
}
