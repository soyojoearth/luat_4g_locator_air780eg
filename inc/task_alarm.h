#ifndef _TASK_ALARM_H
#define _TASK_ALARM_H

#include "luat_rtos.h"


extern luat_rtos_task_handle alarm_task_handle;


void checkAll();

void luat_alarm_task(void *param);


#endif