#include "InlineCurrentSense.h"
#include "main.h"
#include "foc_utils.h"
#include "cmsis_os.h"

/******************************************************************************/
void Current_calibrateOffsets(uint8_t Motor_No);

/******************************************************************************/
void configureADCInline(uint8_t Motor_No,float _shunt_resistor, float _gain,int pinA_,int pinB_,int pinC_ ,ADC_HandleTypeDef *hadcn)
{
	float volts_to_amps_ratio;
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.channel = pinA_;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.hadc = hadcn;
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.channel = pinB_;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.hadc = hadcn;
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.channel = pinC_;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.hadc = hadcn;
	
	volts_to_amps_ratio = 1.0f /_shunt_resistor / _gain; // volts to amps
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.gain =  volts_to_amps_ratio;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.gain = -volts_to_amps_ratio;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.gain =  volts_to_amps_ratio;
	
	printf("gain_a:%.2f,gain_b:%.2f,gain_c:%.2f.\r\n",FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.gain,
						FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.gain,FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.gain);
	
	
}
/******************************************************************************/
void InlineCurrentSense_Init(uint8_t Motor_No,float _shunt_resistor, float _gain,int pinA_,int pinB_,int pinC_,ADC_HandleTypeDef *hadcn)
{
	configureADCInline(Motor_No,_shunt_resistor,_gain,pinA_,pinB_,pinC_,hadcn);
	Current_calibrateOffsets(Motor_No);   //检测偏置电压，也就是电流0A时的运放输出电压值，理论值=1.65V
}
/******************************************************************************/
static float _readADCVoltageInline(ADC_Pin_t _pin)
{
	return ADC_Get_Average_VoltageInline(_pin.hadc,(uint32_t)_pin.channel);
}
/******************************************************************************/
// Function finding zero offsets of the ADC
void Current_calibrateOffsets(uint8_t Motor_No)
{
	int i;
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.I_offset=0;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.I_offset=0;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.I_offset=0;
	// read the adc voltage 1000 times ( arbitrary number )
	for(i=0; i<1000; i++)
	{
		FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.I_offset += _readADCVoltageInline(FOC_Para_M[Motor_No].Inline_Current_Param_.pinA);
		FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.I_offset += _readADCVoltageInline(FOC_Para_M[Motor_No].Inline_Current_Param_.pinB);
		if(_isset(FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.channel)) FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.I_offset += _readADCVoltageInline(FOC_Para_M[Motor_No].Inline_Current_Param_.pinC);
		vTaskDelay(1);
	}
	// calculate the mean offsets
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.I_offset = FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.I_offset/1000;
	FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.I_offset = FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.I_offset/1000;
	if(_isset(FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.channel)) FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.I_offset = 
					FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.I_offset / 1000;
	
	printf("offset_ia:%.4f,offset_ib:%.4f,offset_ic:%.4f.\r\n",FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.I_offset
												,FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.I_offset,FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.I_offset);
}
/******************************************************************************/
// read all three phase currents (if possible 2 or 3)
void getPhaseCurrents(uint8_t Motor_No)
{
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.current.a= 
							(_readADCVoltageInline(FOC_Para_M[Motor_No].Inline_Current_Param_.pinA) - 
							FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.I_offset)*FOC_Para_M[Motor_No].Inline_Current_Param_.pinA.gain;// amps
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.current.b= 
							(_readADCVoltageInline(FOC_Para_M[Motor_No].Inline_Current_Param_.pinB) - 
							FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.I_offset)*FOC_Para_M[Motor_No].Inline_Current_Param_.pinB.gain;// amps
	
	FOC_Para_M[Motor_No].Inline_Current_Param_.current.c = 
							(!_isset(FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.channel)) ? 0 : (_readADCVoltageInline(FOC_Para_M[Motor_No].Inline_Current_Param_.pinC) - 
							FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.I_offset)*FOC_Para_M[Motor_No].Inline_Current_Param_.pinC.gain; // amps
	
}
/******************************************************************************/

