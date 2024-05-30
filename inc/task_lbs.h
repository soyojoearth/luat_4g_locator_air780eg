#ifndef _TASK_LBS_H
#define _TASK_LBS_H

#include "luat_rtos.h"

extern uint8_t g_link_status;

extern double lbs_lat;
extern double lbs_lon;
extern double lbs_accuracy;//米

extern int16_t mobile_rssi;


extern luat_rtos_task_handle lbsLoc_request_task_handle;


void lbsloc_request_task(void *param);

#endif