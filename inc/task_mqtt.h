#ifndef _TASK_MQTT_H
#define _TASK_MQTT_H

#include "commontypedef.h"

extern char* mqttUser;
extern char* mqttPwd;
extern char* mqttDeviceId;

extern char* mqtt_sub_topic;
extern char* mqtt_pub_topic;

extern luat_rtos_task_handle mqtt_task_handle;

void luat_mqtt_task(void *param);

#endif