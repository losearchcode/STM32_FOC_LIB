#ifndef __TIMECOUNT_H__
#define __TIMECOUNT_H__

#ifdef __cplusplus
extern "C" {
#endif
	
#include "stm32f4xx_hal.h"




typedef struct {
    uint32_t xLastWakeTime_t;
		uint32_t xCurrentTime_t;
		uint32_t xElapsedTime_t;
} Interval_Timetick_t;


void First_Time_Laod(Interval_Timetick_t * Interval_Timetick_Point);
uint32_t Get_xCurrentTime_t(Interval_Timetick_t * Interval_Timetick_Point);
uint32_t Get_Interval_Timetick(Interval_Timetick_t * Interval_Timetick_Point);



#ifdef __cplusplus
}
#endif

#endif /*__ GPIO_H__ */

