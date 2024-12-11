#include "main.h" 
#include "cmsis_os.h"

Interval_Timetick_t FOC_Task_Timecount;

void First_Time_Laod(void * Parameter,uint32_t xLastWakeTime)
{
	Interval_Timetick_t *Interval_Timetick_Point = (Interval_Timetick_t *) Parameter;
	
	Interval_Timetick_Point->xLastWakeTime_t = xLastWakeTime;

}

uint32_t Get_Interval_Timetick(void * Parameter)
{
	Interval_Timetick_t *Interval_Timetick_Point = (Interval_Timetick_t *) Parameter;
	// ��ȡ��ǰ�δ���
	uint32_t xCurrentTime = xTaskGetTickCount();

	// �������ϴλ���������ʱ����
	uint32_t xElapsedTime = xCurrentTime - Interval_Timetick_Point->xLastWakeTime_t;
	
	Interval_Timetick_Point->xLastWakeTime_t = xCurrentTime;
	
	return xElapsedTime;
}
