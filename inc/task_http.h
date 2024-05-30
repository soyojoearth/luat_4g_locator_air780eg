#ifndef _TASK_HTTP_H
#define _TASK_HTTP_H

#include "commontypedef.h"

extern luat_rtos_task_handle http_task_handle;

void luat_http_task(void *param);

#endif