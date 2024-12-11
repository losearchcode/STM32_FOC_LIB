#ifndef __TIMECOUNT_H__
#define __TIMECOUNT_H__

#include "main.h"

typedef struct {
    uint32_t xLastWakeTime_t;
} Interval_Timetick_t;

extern Interval_Timetick_t FOC_Task_Timecount;

void First_Time_Laod(void * Parameter,uint32_t xLastWakeTime);
uint32_t Get_Interval_Timetick(void * Parameter);

#endif /*__ GPIO_H__ */

