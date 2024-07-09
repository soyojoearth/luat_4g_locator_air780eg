#include "common_api.h"
#include "luat_rtos.h"
#include "luat_mobile.h"
#include "lbsLoc.h"
#include "luat_network_adapter.h"
#include "luat_debug.h"
#include "luat_mobile.h"
#include "net_lwip.h"

#include "task_gnss.h"

#include "task_lbs.h"

#include "task_alarm.h"

#include "task_http.h"

#include "task_mqtt.h"


static void mobile_event_callback_t(LUAT_MOBILE_EVENT_E event, uint8_t index, uint8_t status)
{
    switch (event)
    {
    case LUAT_MOBILE_EVENT_NETIF:
        switch (status)
        {
        case LUAT_MOBILE_NETIF_LINK_ON:
            g_link_status = 1;
            LUAT_DEBUG_PRINT("网络注册成功\r\n");
			luat_socket_check_ready(index, NULL);
            break;
        default:
            LUAT_DEBUG_PRINT("网络未注册成功\r\n");
            g_link_status = 0;
            break;
        }
    case LUAT_MOBILE_EVENT_SIM:
        switch (status)
        {
        case LUAT_MOBILE_SIM_READY:
            LUAT_DEBUG_PRINT("SIM卡已插入\r\n");
            break;
        case LUAT_MOBILE_NO_SIM:
            LUAT_DEBUG_PRINT("未插SIM卡0，准备切换到sim卡1\r\n");
            luat_mobile_set_sim_id (1);
            break;
        default:
            break;
        }
    case LUAT_MOBILE_EVENT_CELL_INFO:
        switch (status)
        {
        case LUAT_MOBILE_CELL_INFO_UPDATE:
            LUAT_DEBUG_PRINT("周期性搜索小区信息完成一次\r\n");
            break;
        default:
            break;
        }
    }
}


void lbsloc_demo_init(void)
{
    luat_rtos_task_handle lbsloc_demo_task_handle;
    
    luat_rtos_task_create(&lbsLoc_request_task_handle, 4 * 2048, 50, "lbs_task", lbsloc_request_task, NULL, NULL);


    luat_rtos_task_create(&gnss_parse_task_handle, 1024 * 20, 30, "gnss_parse", gnss_parse_task, NULL, 10);
    luat_rtos_task_create(&gnss_task_handle, 1024 * 20, 20, "gnss_task", gnss_setup_task, NULL, NULL);

    //告警检测任务
    luat_rtos_task_create(&alarm_task_handle, 4 * 1024, 50, "alarm_task", luat_alarm_task, NULL, NULL);

    //每次上电，HTTP获取MQTT账号
    luat_rtos_task_create(&http_task_handle, 4 * 1024, 20, "http_task", luat_http_task, NULL, 16);

    // MQTT任务
	luat_rtos_task_create(&mqtt_task_handle, 2 * 1024, 10, "libemqtt", luat_mqtt_task, NULL, 16);
    
}
void mobile_event_register(void)
{
    net_lwip_init();
	net_lwip_register_adapter(NW_ADAPTER_INDEX_LWIP_GPRS);
	network_register_set_default(NW_ADAPTER_INDEX_LWIP_GPRS);
    luat_mobile_event_register_handler(mobile_event_callback_t);
}
INIT_HW_EXPORT(mobile_event_register, "0");
INIT_TASK_EXPORT(lbsloc_demo_init,"1");
