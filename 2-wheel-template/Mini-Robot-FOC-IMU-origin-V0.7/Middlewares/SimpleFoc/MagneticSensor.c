#include "MagneticSensor.h"
#include "main.h"
#include "BLDCMotor.h"

/******************************************************************************/
#define  AS5600_Read_Address  0x6c
#define  AS5600_Address  			0X36
#define  RAW_Angle_Hi    			0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    		0x0D
#define  AS5600_CPR      			4096
/******************************************************************************/
uint16_t I2C_getRawCount(uint8_t Motor_No)
{
	uint8_t I2C_Buffer_Read[4] ={0};
  uint16_t readValue = 0;

	HAL_I2C_Mem_Read(FOC_Para_M[Motor_No].MagneticSensor_.hi2c,AS5600_Read_Address,RAW_Angle_Hi,I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Read,4,50);
	
  int _bit_resolution=12;
  int _bits_used_msb=11-7;
  float cpr = pow(2, _bit_resolution);
  int lsb_used = _bit_resolution - _bits_used_msb;

  uint8_t lsb_mask = (uint8_t)( (1 << lsb_used) - 1 );
  uint8_t msb_mask = (uint8_t)( (1 << _bits_used_msb) - 1 );
  
  readValue = ( I2C_Buffer_Read[1] &  lsb_mask );
  readValue += ( ( I2C_Buffer_Read[0] & msb_mask ) << lsb_used );
	
//	HAL_I2C_Mem_Read(&hi2c3,0x6c,0x0C,I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Read,4,50);
//  
//  readValue = ((uint16_t)I2C_Buffer_Read[0] << 8) | (uint16_t)I2C_Buffer_Read[1];
		
  return readValue; 
}
/******************************************************************************/


#if TLE5012B
/******************************************************************************/
//TLE5012B
#define READ_ANGLE_VALUE  0x8020
#define TLE5012B_CPR      32768
/******************************************************************************/
#define SPI2_CS1_L  GPIO_ResetBits(GPIOB, GPIO_Pin_8)      //CS1_L
#define SPI2_CS1_H  GPIO_SetBits(GPIOB, GPIO_Pin_8)        //CS1_H

#define SPI2_TX_OFF {GPIOB.CRH&=0x0FFFFFFF;GPIOB.CRH|=0x40000000;}  //把PB15(MOSI)配置为浮空输入模式
#define SPI2_TX_ON  {GPIOB.CRH&=0x0FFFFFFF;GPIOB.CRH|=0xB0000000;}  //把PB15(MOSI)复用推挽输出(50MHz)
/******************************************************************************/
unsigned short SPIx_ReadWriteByte(unsigned short byte)
{
	unsigned short retry = 0;
	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		if(++retry>200)return 0;
	}
	SPI_I2S_SendData(SPI2, byte);
	retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) 
	{
		if(++retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI2);
}
/******************************************************************************/
unsigned short ReadTLE5012B_1(unsigned short Comm)
{
	unsigned short u16Data;
	
	SPI2_CS1_L;
	SPIx_ReadWriteByte(Comm);
	SPI2_TX_OFF;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();  //Twr_delay=130ns min
	u16Data = SPIx_ReadWriteByte(0xffff);
	
	SPI2_CS1_H;
	SPI2_TX_ON;
	return(u16Data);
}
/******************************************************************************/
#endif

/******************************************************************************/
void MagneticSensor_Init(uint8_t Motor_No ,I2C_HandleTypeDef* hi2c )
{
#if M1_AS5600
	FOC_Para_M[Motor_No].MagneticSensor_.hi2c = hi2c;
	FOC_Para_M[Motor_No].MagneticSensor_.Cpr=AS5600_CPR;
	FOC_Para_M[Motor_No].MagneticSensor_.Angle_Data_Prev = I2C_getRawCount(Motor_No);  
#elif M1_TLE5012B
	cpr=TLE5012B_CPR;
	angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
	FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset = 0;
	FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev = 0;
	FOC_Para_M[Motor_No].MagneticSensor_.Angle_Data_Prev = 0;
	First_Time_Laod(&FOC_Para_M[Motor_No].MagneticSensor_.AS5600_Vel_Timecount);
}

uint16_t getRawCount(uint8_t Motor_No)  //获取编码器的原始值
{
	uint16_t val;
	
#if M1_AS5600
	val = I2C_getRawCount(Motor_No); 
#elif M1_AS5047P 
	val = ReadAS5047P(Motor_No)&0x3FFF;
#elif M1_TLE5012B
	val = ReadTLE5012B(Motor_No)&0x7FFF;
#elif M1_MA730
	val = ReadMA730(Motor_No);   //高位在前，低位补0
#endif
	
	return val;
}


/******************************************************************************/
float getAngle(uint8_t Motor_No)
{
	long angle_data,d_angle;
	
#if M1_AS5600
	angle_data = getRawCount(Motor_No);
#elif M1_TLE5012B
	angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - FOC_Para_M[Motor_No].MagneticSensor_.Angle_Data_Prev;
	// if overflow happened track it as full rotation
	if(labs(d_angle) > (0.8*FOC_Para_M[Motor_No].MagneticSensor_.Cpr) ) FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset += d_angle > 0 ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	FOC_Para_M[Motor_No].MagneticSensor_.Angle_Data_Prev = angle_data;
	
	
		if(FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset >= ( _2PI*1000)) 
			//转动圈数过多后浮点数精度下降，并导致堵转，每隔一千圈归零一次
	{                                        
		//这个问题针对电机长时间连续转动；如果不是长时间一个方向转动也可以屏蔽掉这几句
		FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset = 0;
		FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev = FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev - _2PI*1000;
	}
	if(FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset <= (-_2PI*1000))
	{
		FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset = 0;
		FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev = FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev + _2PI*1000;
	}
	
	
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return  (FOC_Para_M[Motor_No].MagneticSensor_.Full_Rotation_Offset + ( (float)angle_data / (float)FOC_Para_M[Motor_No].MagneticSensor_.Cpr) * _2PI) ;
}
/******************************************************************************/
// Shaft velocity calculation
float getVelocity(uint8_t Motor_No)
{
	float Ts, angle_c, vel;

	Ts = (float)Get_Interval_Timetick(&FOC_Para_M[Motor_No].MagneticSensor_.AS5600_Vel_Timecount)*Tick_to_Time_Scale;
	if(Ts == 0.0f || Ts > 0.5f) Ts = Tick_to_Time_Scale; 

	// current angle
	angle_c = FOC_Para_M[Motor_No].Motor_State_.shaft_angle;
	//angle_c = getAngle(Motor_No);
	// velocity calculation
	vel = (angle_c - FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev)/Ts;

	// save variables for future pass
	FOC_Para_M[Motor_No].MagneticSensor_.Angle_Prev = angle_c;

	return vel;
}
/******************************************************************************/



