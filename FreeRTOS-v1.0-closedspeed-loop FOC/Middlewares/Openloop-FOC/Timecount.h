#ifndef __TIMECOUNT_H__
#define __TIMECOUNT_H__

#ifdef __cplusplus
extern "C" {
#endif
	

#include "main.h"


#define Task_Timecount_Num FOC_Motor_Num


typedef struct {
    uint32_t xLastWakeTime_t;
		uint32_t xCurrentTime_t;
} Interval_Timetick_t;

extern Interval_Timetick_t FOC_Task_Timecount[Task_Timecount_Num];

void First_Time_Laod(void * Parameter);
uint32_t Get_xCurrentTime_t(void * Parameter);
uint32_t Get_Interval_Timetick(void * Parameter);



#ifdef __cplusplus
}
#endif

#endif /*__ GPIO_H__ */

