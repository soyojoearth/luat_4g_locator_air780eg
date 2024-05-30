#ifndef __AGNSS_H__
#define __AGNSS_H__
// #define EPH_TIME_FILE "/ephTime.txt"
// #define EPH_FILE_PATH "/ephData.bin"

#include "luat_rtos.h"

extern luat_rtos_task_handle gnss_task_handle;
extern luat_rtos_task_handle gnss_parse_task_handle;

extern double gnss_lat;//最近一次gnss定位
extern double gnss_lon;
extern double gnss_accuracy;//米

void gnss_setup_task(void *param);

void gnss_parse_task(void *param);

#endif