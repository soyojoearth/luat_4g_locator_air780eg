#include "luat_network_adapter.h"
#include "common_api.h"
#include "luat_debug.h"
#include "luat_mem.h"
#include "luat_rtos.h"
#include "luat_mobile.h"


#include "luat_gpio.h"
#include "networkmgr.h"
#include "luat_http.h"

#include "cJSON.h"



#include "task_lbs.h"

#include "global.h"

#include "task_alarm.h"

#include "task_mqtt.h"

#include "task_http.h"


enum
{
	TEST_HTTP_GET_HEAD = 0,
	TEST_HTTP_GET_HEAD_DONE,
	TEST_HTTP_GET_DATA,
	TEST_HTTP_GET_DATA_DONE,
	TEST_HTTP_POST_DATA_START,
	TEST_HTTP_POST_DATA_CONTINUE,
	TEST_HTTP_FAILED,
};

luat_rtos_task_handle http_task_handle;

static void luatos_http_cb(int status, void *data, uint32_t len, void *param)
{
	uint8_t *head_data;
	uint8_t *body_data;
    if(status < 0) 
    {
		luat_rtos_event_send(param, TEST_HTTP_FAILED, 0, 0, 0, 0);
		return;
    }
	switch(status)
	{
	case HTTP_STATE_GET_BODY:
		if (data)
		{
			body_data = luat_heap_malloc(len);
			memcpy(body_data, data, len);
			luat_rtos_event_send(param, TEST_HTTP_GET_DATA, (uint32_t)body_data, len, 0, 0);
		}
		else
		{
			luat_rtos_event_send(param, TEST_HTTP_GET_DATA_DONE, 0, 0, 0, 0);
		}
		break;
	case HTTP_STATE_GET_HEAD:
		if (data)
		{
			head_data = luat_heap_malloc(len);
			memcpy(head_data, data, len);
			luat_rtos_event_send(param, TEST_HTTP_GET_HEAD, (uint32_t)head_data, len, 0, 0);
		}
		else
		{
			luat_rtos_event_send(param, TEST_HTTP_GET_HEAD_DONE, 0, 0, 0, 0);
		}
		break;
	case HTTP_STATE_IDLE:
		break;
	case HTTP_STATE_SEND_BODY_START:
		//如果是POST，在这里发送POST的body数据，如果一次发送不完，可以在HTTP_STATE_SEND_BODY回调里继续发送
		luat_rtos_event_send(param, TEST_HTTP_POST_DATA_START, 0, 0, 0, 0);
		break;
	case HTTP_STATE_SEND_BODY:
		//如果是POST，可以在这里发送POST剩余的body数据
		luat_rtos_event_send(param, TEST_HTTP_POST_DATA_CONTINUE, 0, 0, 0, 0);
		break;
	default:
		break;
	}
}

void luat_http_task(void *param)
{
	

	while (strlen(mqttUser) == 0 || strlen(mqttPwd) == 0 || strlen(mqttDeviceId) == 0){

		uint8_t ret = 0;
		luat_event_t event = {0};
		uint8_t is_end = 0;
		uint32_t done_len = 0;
		luat_http_ctrl_t *http = luat_http_client_create(luatos_http_cb, luat_rtos_get_current_handle(), -1);
		luat_http_client_ssl_config(http, 0, NULL, 0, NULL, 0, NULL, 0, NULL, 0);
		char remote_domain[200];
		char post_data[200];

		char IMEI_STR[16] = {0};

		//一定要有产测账号，才能自动获取到MQTT授权码，请向我们索取！微信: napshen （申工）
		while(strlen(facId) == 0 || strlen(facSecret) == 0)
		{
			LUAT_DEBUG_PRINT("Please ask us for the IoT Account!!!! Weixin: napshen ");
			luat_rtos_task_sleep(1000);
		}

		ret = luat_mobile_get_imei(0, IMEI_STR, sizeof(IMEI_STR)-1);

		if(ret <= 0){
			while(1)
			{
				LUAT_DEBUG_PRINT("can not get imei : error luat_mobile_get_imei()");
				luat_rtos_task_sleep(1000);
			}
		}

		snprintf((char *)remote_domain, 200, "%s", "https://iot-saas-admin.newxton.com/api/factory_device_mqtt_account_renew");
		
		snprintf((char *)post_data, 200, "{\"facId\":\"%s\", \"facSecret\":\"%s\",\"clientId\":\"%s\",\"clientType\":\"%s\"}", facId, facSecret, IMEI_STR, clientType);//每个字段一定要带引号

		char tmp[16];
		sprintf_(tmp, "%d", strlen(post_data));
		luat_http_client_set_user_head(http, "Content-Type", "application/json");
		luat_http_client_set_user_head(http, "Content-Length", tmp);

		LUAT_DEBUG_PRINT("print url %s", remote_domain);
		luat_http_client_start(http, remote_domain, 1, 0, 0);
		while (!is_end)
		{
			luat_rtos_event_recv(http_task_handle, 0, &event, NULL, LUAT_WAIT_FOREVER);
			LUAT_DEBUG_PRINT("event %d", event.id);
			switch(event.id)
			{
			case TEST_HTTP_GET_HEAD:
				luat_heap_free((char *)event.param1);
				break;
			case TEST_HTTP_GET_HEAD_DONE:
				// 在这里处理http响应头
				done_len = 0;
				LUAT_DEBUG_PRINT("status %d total %u", luat_http_client_get_status_code(http), http->total_len);
				break;
			case TEST_HTTP_GET_DATA:
				// 在这里处理用户数据
				done_len += event.param2;
				LUAT_DEBUG_PRINT("%.*s", event.param2, event.param1);

				cJSON *json = cJSON_Parse(event.param1);
				cJSON *cJson_mqttUser = NULL;
				cJSON *cJson_mqttPwd = NULL;
				cJSON *cJson_mqttDeviceId = NULL;
				
				if (json != NULL)
				{
					cJson_mqttUser = cJSON_GetObjectItemCaseSensitive(json, "mqttUser");
					cJson_mqttPwd = cJSON_GetObjectItemCaseSensitive(json, "mqttPwd");
					cJson_mqttDeviceId = cJSON_GetObjectItemCaseSensitive(json, "mqttDeviceId");

					if (cJSON_IsString(cJson_mqttUser) && (cJson_mqttUser->valuestring != NULL) && 
						cJSON_IsString(cJson_mqttPwd) && (cJson_mqttPwd->valuestring != NULL) && 
						cJSON_IsString(cJson_mqttDeviceId) && (cJson_mqttDeviceId->valuestring != NULL))
					{

						LUAT_DEBUG_PRINT("mqttUser: %s\n", cJson_mqttUser->valuestring);
						LUAT_DEBUG_PRINT("mqttPwd: %s\n", cJson_mqttPwd->valuestring);
						LUAT_DEBUG_PRINT("mqttDeviceId: %s\n", cJson_mqttDeviceId->valuestring);

						mqttUser=(char*)malloc(strlen(cJson_mqttUser->valuestring)+1);
						strcpy(mqttUser,cJson_mqttUser->valuestring);

						mqttPwd=(char*)malloc(strlen(cJson_mqttPwd->valuestring)+1);
						strcpy(mqttPwd,cJson_mqttPwd->valuestring);

						mqttDeviceId=(char*)malloc(strlen(cJson_mqttDeviceId->valuestring)+1);
						strcpy(mqttDeviceId,cJson_mqttDeviceId->valuestring);
						
					}
				}

				cJSON_Delete(json);

				luat_heap_free((char *)event.param1);
				break;
			case TEST_HTTP_GET_DATA_DONE:
				is_end = 1;
				break;
			case TEST_HTTP_POST_DATA_START:
				luat_http_client_post_body(http, (void *)post_data, strlen(post_data));
				break;
			case TEST_HTTP_POST_DATA_CONTINUE:
				break;
			case TEST_HTTP_FAILED:
				is_end = 1;
				break;
			default:
				break;
			}
		}
		LUAT_DEBUG_PRINT("http test end, total count %d", done_len);
		luat_http_client_close(http);
		luat_http_client_destroy(&http);
		
		luat_rtos_task_sleep(30000);
	}

	while (1)
	{
		luat_rtos_task_sleep(60000);
	}

}