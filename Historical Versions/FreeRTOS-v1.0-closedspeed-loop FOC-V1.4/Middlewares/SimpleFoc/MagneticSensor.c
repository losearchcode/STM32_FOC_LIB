#include "MagneticSensor.h"

/******************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/

/******************************************************************************/
#define  AS5600_Read_Address  0x6c
#define  AS5600_Address  			0X36
#define  RAW_Angle_Hi    			0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    		0x0D
#define  AS5600_CPR      			4096
/******************************************************************************/
unsigned short I2C_getRawCount(I2C_HandleTypeDef * I2Cx)
{
	uint8_t I2C_Buffer_Read[4] ={0};
  uint16_t readValue = 0;

	HAL_I2C_Mem_Read(I2Cx,AS5600_Read_Address,RAW_Angle_Hi,I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Read,4,50);
	
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

#define SPI2_TX_OFF {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0x40000000;}  //把PB15(MOSI)配置为浮空输入模式
#define SPI2_TX_ON  {GPIOB->CRH&=0x0FFFFFFF;GPIOB->CRH|=0xB0000000;}  //把PB15(MOSI)复用推挽输出(50MHz)
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
void MagneticSensor_Init(void)
{
#if M1_AS5600
	cpr=AS5600_CPR;
	angle_data_prev = I2C_getRawCount(&hi2c3);  
#elif M1_TLE5012B
	cpr=TLE5012B_CPR;
	angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
	full_rotation_offset = 0;
	velocity_calc_timestamp=0;
}
/******************************************************************************/
float getAngle(void)
{
	float angle_data,d_angle;
	
#if M1_AS5600
	angle_data = I2C_getRawCount(&hi2c3);
#elif M1_TLE5012B
	angle_data = ReadTLE5012B_1(READ_ANGLE_VALUE)&0x7FFF;
#endif
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*cpr) ) full_rotation_offset += d_angle > 0 ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev = angle_data;
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return  (full_rotation_offset + ( angle_data / (float)cpr) * _2PI) ;
}
/******************************************************************************/
// Shaft velocity calculation
float getVelocity(void)
{
	unsigned long now_us;
	float Ts, angle_c, vel;

	now_us = HAL_GetTick(); //_micros();
	if(now_us>=velocity_calc_timestamp)Ts = (float)(now_us - velocity_calc_timestamp)/1e-3;
	else
		Ts = (float)(0xFFFFFF - velocity_calc_timestamp + now_us)/1e-3;
	velocity_calc_timestamp=now_us;  //save timestamp for next call
  // quick fix for strange cases (micros overflow)
  if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 

	// current angle
	angle_c = getAngle();
	// velocity calculation
	vel = (angle_c - angle_prev)/Ts;

	// save variables for future pass
	angle_prev = angle_c;
	velocity_calc_timestamp = now_us;
	return vel;
}
/******************************************************************************/


