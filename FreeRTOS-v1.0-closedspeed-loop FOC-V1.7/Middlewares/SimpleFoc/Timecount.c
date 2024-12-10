#include "Timecount.h" 
#include "main.h"
#include "cmsis_os.h"



void First_Time_Laod(Interval_Timetick_t * Interval_Timetick)
{
	uint32_t xLastWakeTime = xTaskGetTickCount();
	Interval_Timetick->xCurrentTime_t = xLastWakeTime;
	Interval_Timetick->xLastWakeTime_t	=	Interval_Timetick->xCurrentTime_t;

}

uint32_t Get_xCurrentTime_t(Interval_Timetick_t * Interval_Timetick_Point)
{
	
	Interval_Timetick_Point->xCurrentTime_t = xTaskGetTickCount();
	
	return Interval_Timetick_Point->xCurrentTime_t;
}
	

uint32_t Get_Interval_Timetick(Interval_Timetick_t * Interval_Timetick_Point)
{
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
	
	Interval_Timetick_Point->xElapsedTime_t = xElapsedTime;

	Interval_Timetick_Point->xLastWakeTime_t = xCurrentTime;
	
	
	
	return xElapsedTime;
}
