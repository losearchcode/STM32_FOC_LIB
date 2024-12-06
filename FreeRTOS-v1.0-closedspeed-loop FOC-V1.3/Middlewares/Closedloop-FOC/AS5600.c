//�Ƹ翪Դ����ѭGNUЭ�飬ת����������Ȩ��
//GNU��ԴЭ�飨GNU General Public License, GPL����һ������������Э�飬�����û��ܹ����ɵ�ʹ�á��о���������޸������
//��Э�����Ҫ�ص��ǣ�Ҫ���κ��޸Ļ���������Ʒ��������ͬ�ķ�ʽ���������������뿪Դ�����⣬��Э��ҲҪ����ʹ�û�ַ����ʱ�����뱣����Ȩ��Ϣ�����Э�顣GNU��ԴЭ���������������ᣨFSF���ƶ���ά����һ��Э�飬������GNU�ƻ��������������������С�
//����DengFOC�ٷ�Ӳ���ϲ��Թ�����ӭӲ������/֧�����ߣ��Ա��������̣��Ƹ翪Դ
//���֧�ֽ��ǽ���������Ƶ�ͳ�����Դ�ľ��ѣ��Ƹ���������лл�����


#include "main.h" 
#include "cmsis_os.h"

int _raw_ang_hi = 0x0c;
int _raw_ang_lo = 0x0d;
int _ams5600_Read_Address = 0x6c;
int ledtime = 0;

Sensor_AS5600 Sensor_AS5600_Parameter[AS5600_NUM];

float getVelocity_(Sensor_AS5600* sensor) ;

double getSensorAngle(Sensor_AS5600* sensor)
{
  uint8_t I2C_Buffer_Read[4] ={0};
  uint16_t readValue = 0;

	HAL_I2C_Mem_Read(sensor->hi2c,_ams5600_Read_Address,_raw_ang_hi,I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Read,4,50);
	
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
		
  return (readValue/ (float)cpr) * _2PI; 
}

void Sensor_init(Sensor_AS5600* sensor,int _Mot_Num,I2C_HandleTypeDef* _hi2c) 
{
	sensor->Mot_Num=_Mot_Num;  //ʹ�� Mot_Num ����ͳһ�ڸ��ļ�����
	sensor->hi2c = _hi2c;
	
	getSensorAngle(sensor); 
	vTaskDelay(1);
	sensor->Vel_angle_prev = getSensorAngle(sensor);
	First_Time_Laod(&sensor->AS5600_Vel_Timecount);
	vTaskDelay(1);
	
	getSensorAngle(sensor); 
	vTaskDelay(1);
	sensor->Angle_prev = getSensorAngle(sensor);
	First_Time_Laod(&sensor->AS5600_Angle_Timecount);
	
}

void Sensor_update(Sensor_AS5600* sensor)
{
    float val = getSensorAngle(sensor);
    sensor->Angle_Interval_Ts = Get_Interval_Timetick(&sensor->AS5600_Angle_Timecount);
    float d_angle = val - sensor->Angle_prev;
    // Ȧ�����
    if(fabsf(d_angle) > (0.8f*_2PI) ) sensor->full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    sensor->Angle_prev = val;
		sensor->Current_Vel = getVelocity_(sensor);
}

float getMechanicalAngle(Sensor_AS5600* sensor) 
{
    return sensor->Angle_prev;
}

float getAngle(Sensor_AS5600* sensor)
{
    return (float)sensor->full_rotations * _2PI + sensor->Angle_prev;
}

float getVelocity_(Sensor_AS5600* sensor) 
{
    // �������ʱ��
		sensor->Vel_Interval_ts = sensor->Angle_Interval_Ts;
    float Ts = (sensor->Vel_Interval_ts)*1e-3f;
    // �����޸���ֵ������΢�����
    if(Ts <= 0) Ts = 1e-3f;
    // �ٶȼ���
    float vel = ( (float)(sensor->full_rotations - sensor->Vel_full_rotations)*_2PI 
									+ (sensor->Angle_prev - sensor->Vel_angle_prev) ) / Ts;    
    // ��������Դ�����ʹ��
    sensor->Vel_angle_prev = sensor->Angle_prev;
    sensor->Vel_full_rotations = sensor->full_rotations;
    return vel;
}

float getVelocity(Sensor_AS5600* sensor) 
{
    return sensor->Current_Vel;
}
