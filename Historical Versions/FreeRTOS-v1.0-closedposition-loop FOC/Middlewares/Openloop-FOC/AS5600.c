//�Ƹ翪Դ����ѭGNUЭ�飬ת����������Ȩ��
//GNU��ԴЭ�飨GNU General Public License, GPL����һ������������Э�飬�����û��ܹ����ɵ�ʹ�á��о���������޸������
//��Э�����Ҫ�ص��ǣ�Ҫ���κ��޸Ļ���������Ʒ��������ͬ�ķ�ʽ���������������뿪Դ�����⣬��Э��ҲҪ����ʹ�û�ַ����ʱ�����뱣����Ȩ��Ϣ�����Э�顣GNU��ԴЭ���������������ᣨFSF���ƶ���ά����һ��Э�飬������GNU�ƻ��������������������С�
//����DengFOC�ٷ�Ӳ���ϲ��Թ�����ӭӲ������/֧�����ߣ��Ա��������̣��Ƹ翪Դ
//���֧�ֽ��ǽ���������Ƶ�ͳ�����Դ�ľ��ѣ��Ƹ���������лл�����


#include "main.h" 


int _raw_ang_hi = 0x0c;
int _raw_ang_lo = 0x0d;
int _ams5600_Address = 0x36;
int ledtime = 0;
int32_t full_rotations=0; // full rotation tracking;
float angle_prev=0; 

//double abs_f(double x)
//{
//	if(x<0) x =-x;
//	else x=x;
//	return (float)x;
//}


//readTwoBytes(int in_adr_hi, int in_adr_lo)��δ�����һ����������Ŀ���Ǵ�I2C�豸���ڴ����еı�����Ϊ_ams5600_Address���ж�ȡ�����ֽ����ݣ�������ϲ���һ��16λ���޷����������ء�
//������˵�����������������Ͳ���in_adr_hi��in_adr_lo����������ָ����Ҫ��ȡ�������ֽ����ݵĵ�ַ������������ͨ��Wire�⿪ʼI2C���䣬���豸д��in_adr_lo��in_adr_hi�ֱ���Ϊ���ݵ�ַ��Ȼ���ȡ��Ӧ���ֽ����ݡ�
//��ÿ��Wire.requestFrom()����֮��ͨ��һ��whileѭ���ȴ����ݽ�����ϡ�Ȼ���ȡ���յ��ĵ��ֽں͸��ֽڣ���ʹ��λ���㽫���Ǻϲ���һ��16λ���޷���������
//��󣬷��غϲ���������������ȡ�����г��ִ�����ߺ���û�гɹ���ȡ�����ݣ���������-1��
uint16_t readTwoBytes(void)
{
  uint16_t retVal = 60000;
  uint8_t I2C_Buffer_Read[4] ={0};
  /* ����λ */
  HAL_I2C_Mem_Read(&hi2c3,0x6c,0x0C,I2C_MEMADD_SIZE_8BIT,I2C_Buffer_Read,4,50);
  
  retVal = ((uint16_t)I2C_Buffer_Read[0] << 8) | (uint16_t)I2C_Buffer_Read[1];
  
  return retVal;
}

uint16_t getRawAngle()
{
  return readTwoBytes();
}

float getAngle_Without_track(void)
{
	if(getRawAngle()<5000)
		return getRawAngle()*0.08789* PI / 180;    //�õ������ƵĽǶ�(360/4096(һȦ����))
	else
		return (-65534);
}

float getAngle(void)
{
    float val = getAngle_Without_track();
		FOC_Init_Parameter.xCurrent_electricalAngle = _electricalAngle_AS5600(val);
		if(val == -65534)
			return 0;
		else
		{
			float d_angle = val - angle_prev;
			if(fabsf(d_angle)<=(float)0.0001)d_angle=(float)0.0001;
			//������ת����Ȧ��
			//ͨ���жϽǶȱ仯�Ƿ����80%��һȦ(0.8f*6.28318530718f)���ж��Ƿ������������������ˣ���full_rotations����1�����d_angleС��0�������1�����d_angle����0����
			if(fabsf(d_angle) > (0.9f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
//			if(abs_f(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
			angle_prev = val;
			return (float)full_rotations * 6.28318530718f + angle_prev;
		}
}
