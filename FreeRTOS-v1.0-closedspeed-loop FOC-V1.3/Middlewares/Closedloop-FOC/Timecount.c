#include "main.h" 
#include "cmsis_os.h"

Interval_Timetick_t FOC_Task_Timecount[Task_Timecount_Num];


void First_Time_Laod(void * Parameter)
{
	Interval_Timetick_t *Interval_Timetick_Point = (Interval_Timetick_t *) Parameter;
	
	uint32_t xLastWakeTime = xTaskGetTickCount();
	Interval_Timetick_Point->xLastWakeTime_t = xLastWakeTime;
	Interval_Timetick_Point->xCurrentTime_t = xLastWakeTime;

}

uint32_t Get_xCurrentTime_t(void * Parameter)
{
	Interval_Timetick_t *Interval_Timetick_Point = (Interval_Timetick_t *) Parameter;
	
	Interval_Timetick_Point->xCurrentTime_t = xTaskGetTickCount();
	
	return Interval_Timetick_Point->xCurrentTime_t;
}
	

uint32_t Get_Interval_Timetick(void * Parameter)
{
	Interval_Timetick_t *Interval_Timetick_Point = (Interval_Timetick_t *) Parameter;
	// 读取当前滴答数
	uint32_t xCurrentTime = Get_xCurrentTime_t(Interval_Timetick_Point);

	// 计算自上次唤醒以来的时间间隔
	uint32_t xElapsedTime;
	if(xCurrentTime < Interval_Timetick_Point->xLastWakeTime_t)
	{
		xElapsedTime = portMAX_DELAY - Interval_Timetick_Point->xLastWakeTime_t;
		xElapsedTime += xCurrentTime;
	}
	else
	{
		xElapsedTime = xCurrentTime - Interval_Timetick_Point->xLastWakeTime_t;
	}
	
	Interval_Timetick_Point->xLastWakeTime_t = xCurrentTime;
	
	return xElapsedTime;
}
