#include "luat_network_adapter.h"
#include "common_api.h"
#include "luat_debug.h"
#include "luat_mem.h"
#include "luat_rtos.h"
#include "luat_mobile.h"

#include "libemqtt.h"
#include "luat_mqtt.h"

#include "task_lbs.h"

#include "task_gnss.h"

#include "global.h"

#include "task_alarm.h"

#include "task_mqtt.h"


#define MQTT_DEMO_AUTOCON 		1
#define MQTT_DEMO_PUB_QOS 		0

#define MQTT_HOST    	"iot-mqtt-host.newxton.com"   				// MQTT服务器的地址和端口号
#define MQTT_PORT		1883

char *topic_up_prex = "device/up/";
char *topic_down_prex = "device/down/";

char* mqttUser = "";
char* mqttPwd = "";
char* mqttDeviceId = "";

char* mqtt_sub_topic = "";
char* mqtt_pub_topic = "";

char* mqtt_will_payload = "{\"netStatus\":\"offline\"}";  //遗嘱内容，即时更新离线状态

char mqtt_upload_buffer[1024];//看需要设定大小，其中电子围栏就需要252个字节

char IMEI_STR[16] = {0};

char* identifier = "";//从MQTT接收到 dpId 3，动态分配内存

char* owner = "";//从MQTT接收到 dpId 4，动态分配内存

bool pair_upload = false;

int32_t pairStatus = 1;//10 pairStatus 0：非配网状态；1：等待配网状态   ，一般需要用户 长按3秒 按钮，才能进入等待配网状态（闪灯），默认非配网状态(位了安全)

luat_rtos_task_handle mqtt_task_handle;

void receiveMessage(luat_mqtt_ctrl_t *luat_mqtt_ctrl, uint8_t * message, uint16_t len);
void uploadMessage(luat_mqtt_ctrl_t *luat_mqtt_ctrl);


static void luat_mqtt_cb(luat_mqtt_ctrl_t *luat_mqtt_ctrl, uint16_t event){
	int ret;
	switch (event)
	{
	case MQTT_MSG_CONNACK:{
		LUAT_DEBUG_PRINT("mqtt_connect ok");

		LUAT_DEBUG_PRINT("mqtt_subscribe");
		uint16_t msgid = 0;
		mqtt_subscribe(&(luat_mqtt_ctrl->broker), mqtt_sub_topic, &msgid, 1);

		LUAT_DEBUG_PRINT("publish");
		break;
	}
	case MQTT_MSG_PUBLISH : {
		const uint8_t* ptr;
		uint16_t topic_len = mqtt_parse_pub_topic_ptr(luat_mqtt_ctrl->mqtt_packet_buffer, &ptr);
		LUAT_DEBUG_PRINT("pub_topic: %.*s",topic_len,ptr);
		uint16_t payload_len = mqtt_parse_pub_msg_ptr(luat_mqtt_ctrl->mqtt_packet_buffer, &ptr);
		LUAT_DEBUG_PRINT("pub_msg: %.*s",payload_len,ptr);
		receiveMessage(luat_mqtt_ctrl, ptr, payload_len);//解析数据
		break;
	}
	// case MQTT_MSG_TCP_TX_DONE:
	// 	//如果用QOS0发送，可以作为发送成功的初步判断依据
	// 	if (0 == MQTT_DEMO_PUB_QOS)
	// 	{
	// 		LUAT_DEBUG_PRINT("publish send ok");
	// 	}
	// 	break;
	case MQTT_MSG_PUBACK : 
	case MQTT_MSG_PUBCOMP : {
		LUAT_DEBUG_PRINT("msg_id: %d",mqtt_parse_msg_id(luat_mqtt_ctrl->mqtt_packet_buffer));
		break;
	}
	case MQTT_MSG_RELEASE : {
		LUAT_DEBUG_PRINT("luat_mqtt_cb mqtt release");
		break;
	}
	case MQTT_MSG_DISCONNECT : { // mqtt 断开(只要有断开就会上报,无论是否重连)
		LUAT_DEBUG_PRINT("luat_mqtt_cb mqtt disconnect");
		break;
	}
	case MQTT_MSG_TIMER_PING : {
		luat_mqtt_ping(luat_mqtt_ctrl);
		break;
	}
	case MQTT_MSG_RECONNECT : {
if (MQTT_DEMO_AUTOCON == 1)
{
		luat_mqtt_reconnect(luat_mqtt_ctrl);
}
		break;
	}
	case MQTT_MSG_CLOSE : { // mqtt 关闭(不会再重连)  注意：一定注意和MQTT_MSG_DISCONNECT区别，如果要做手动重连处理推荐在这里 */
		LUAT_DEBUG_PRINT("luat_mqtt_cb mqtt close");
if (MQTT_DEMO_AUTOCON == 0){
	ret = luat_mqtt_connect(luat_mqtt_ctrl);
	if (ret) {
		LUAT_DEBUG_PRINT("mqtt connect ret=%d\n", ret);
		luat_mqtt_close_socket(luat_mqtt_ctrl);
		return;
	}
}
		break;
	}
	default:
		break;
	}
	return;
}


uint16_t CRC16_CCITT_FALSE(char *data, uint16_t len)  
{  
    uint16_t wCRCin = 0xFFFF;  
    uint16_t wCPoly = 0x1021;  
    char wChar = 0;  
    
    while (len--)
    {  
        wChar = *(data++);  
        wCRCin ^= (wChar << 8); 
        
        for(int i = 0; i < 8; i++)  
        {  
            if(wCRCin & 0x8000)  
            {
                wCRCin = (wCRCin << 1) ^ wCPoly;  
            }
            else  
            {
                wCRCin = wCRCin << 1; 
            }            
        }  
    }  
    return (wCRCin) ;  
}

void receiveMessage(luat_mqtt_ctrl_t *luat_mqtt_ctrl, uint8_t * data, uint16_t len){

	//收到通信协议后对比协议头，然后再核实CRC，data是收到的数据
	if (data[0] == (uint8_t)0xAA && data[1] == (uint8_t)0x55)
	{
		LUAT_DEBUG_PRINT("Nxt Protocol D-TLV \n");
		//Nxt Protocol D-TLV
		uint16_t length = (data[3] & 0x00ff)<<8 | (data[4] & 0x00ff);

		LUAT_DEBUG_PRINT("Nxt Protocol length %d \n",length);
		//CRC
		uint16_t length_crc = length + 2 + 1;
		uint16_t crc_index = length + 2 + 1 + 2;
		uint16_t crc_data = (data[crc_index] & 0x00ff)<<8 | (data[crc_index+1] & 0x00ff);

		LUAT_DEBUG_PRINT("Nxt Protocol length_crc:%d \n",length_crc);

		uint16_t crc_result = CRC16_CCITT_FALSE(&(data[2]), length_crc);
		uint8_t crc_result_1 = (crc_result >> 8) & 0xff;
		uint8_t crc_result_2 = crc_result & 0xff;

		LUAT_DEBUG_PRINT("Nxt Protocol 3 \n");
		LUAT_DEBUG_PRINT("Nxt Protocol 3 \n");

		if (crc_result != crc_data)
		{
			LUAT_DEBUG_PRINT("crc error %02x  %02x \n",crc_result_1,crc_result_2);
		}
		else{
			LUAT_DEBUG_PRINT("crc ok %02x  %02x \n",crc_result_1,crc_result_2);

			int last_payload_length = length;
			int index_payload = 5;

			uint8_t item_dpid;
			uint8_t item_type;
			uint16_t item_len;
			uint16_t index_value;			

			while (last_payload_length >= 4)
			{
				item_dpid = data[index_payload + 0];
				item_type = data[index_payload + 1];
				item_len = ((data[index_payload + 2] & 0x00ff) << 8) | (data[index_payload + 3] & 0x00ff);
				index_value = index_payload + 4;

				//pairStatus 0：非配网状态；1：等待配网状态   ，一般需要用户 长按3秒 按钮，才能进入等待配网状态（闪灯），默认非配网状态(位了安全)
				//但我们这里用另一种方式：开机或reset后大约10秒到2分钟内，pairStatus是1，超过2分钟后自动变成0，这样也还比较安全
				//当pairStatus不等于1时，忽略配网绑定请求
				if(pairStatus == 1){
					//设备绑定请求
					if(item_dpid == 3){//identifier
						identifier = (char*)malloc(item_len+1);
						for (size_t i = 0; i < item_len; i++)
						{
							identifier[i] = data[index_value+i];
						}
						identifier[item_len] = 0x00;			
						pair_upload = true;
					}
					else if(item_dpid == 4){//owner
						owner = (char*)malloc(item_len+1);
						for (size_t i = 0; i < item_len; i++)
						{
							owner[i] = data[index_value+i];
						}
						owner[item_len] = 0x00;		
						pair_upload = true;
					}
				}
				
				//普通数据
				if(item_dpid == 100){//警报开关
					//bitmap 
					dpValue_alarmFlag = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);
				}
				else if(item_dpid == 158){//启用短信告警
					//bitmap 
					dpValue_alarmFlagSms = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);
				}
				else if (item_dpid == 104)//电子围栏数据
				{
					dpValue_data_geofencing_length = item_len;
					for (size_t i = 0; i < 252; i++)
					{
						if (i < item_len){
							dpValue_data_geofencing[i] = data[index_value+i];
						}
						else
						{
							dpValue_data_geofencing[i] = 0x00;
						}
					}
				}
				else if(item_dpid == 120){
					//bool 举例
					dpValue_power_on = data[index_value++] & 0xff;

				}
				else if(item_dpid == 121){

					dpValue_frequency = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 110){

					dpValue_PascalMaxLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 111){

					dpValue_PascalMinLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 112){

					dpValue_tempMaxLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 113){

					dpValue_tempMinLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 114){

					dpValue_humMaxLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 115){

					dpValue_humMinLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				else if(item_dpid == 142){

					dpValue_speedLimit = ((data[index_value] & 0xffffffff) << 24) | 
					((data[index_value+1] & 0xffffffff) << 16) | 
					((data[index_value+2] & 0xffffffff) << 8) | 
					((data[index_value+3] & 0xffffffff) << 0);

				}
				
				last_payload_length = last_payload_length - 4 - item_len;
				index_payload = index_payload + item_len + 4;				

			}
			
			//其它处理。。。

			//收到所有数据，并且处理完毕后，重新上报一下数据
            check_and_upload_once = 1;
			upload_all_once = true;
		}
		
	}
}


void uploadMessage(luat_mqtt_ctrl_t *luat_mqtt_ctrl){

	int ret = -1;

	//Protocol Head: 0xAA 0x55
	mqtt_upload_buffer[0] = 0xAA;
	mqtt_upload_buffer[1] = 0x55;

	//Protocol Version
	mqtt_upload_buffer[2] = 0x02;

	//payload length 到后面再补上
	// mqtt_upload_buffer[3] = 0x00;
	// mqtt_upload_buffer[4] = 0x00;

	uint16_t index = 5;

	//ProductId
	mqtt_upload_buffer[index++] = 0x01;
	mqtt_upload_buffer[index++] = 0x03;
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = strlen(product_id) & 0xff;

	for (size_t i = 0; i < strlen(product_id); i++)
	{
		mqtt_upload_buffer[index++] = product_id[i];
	}

	//**********************************下面这两条是绑定设备到用户，仅作为开发调试使用，正式情况下是按照文档上的流程发送这两条数据************************//

	if(pair_upload && strlen(identifier) > 0 && strlen(owner) > 0 ){

		//3 校验标识码（string）
		mqtt_upload_buffer[index++] = 0x03;
		mqtt_upload_buffer[index++] = 0x03;
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = strlen(identifier) & 0xff;//每次owner变化的时候，identifier要重新随机生成一次，当owner不变，identifier就绝不能变

		for (size_t i = 0; i < strlen(identifier); i++)
		{
			mqtt_upload_buffer[index++] = identifier[i];
		}

		//4 绑定的用户名（string）
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = 0x03;
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = strlen(owner) & 0xff;//就是被绑定用户的用户名，app会通过mqtt发给设备的

		for (size_t i = 0; i < strlen(owner); i++)
		{
			mqtt_upload_buffer[index++] = owner[i];
		}

		pairStatus = 0;//及时关闭可绑定状态

	}

	pair_upload = false;//发一次就够了。每次绑定请求的时候，发一次。

	//***********************************上面这两条是绑定设备到用户，仅作为开发调试使用，正式情况下是按照文档上的流程发送这两条数据************************//


	//10 pairStatus 0：非配网状态；1：等待配网状态   
	mqtt_upload_buffer[index++] = 10 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (pairStatus >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (pairStatus >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (pairStatus >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (pairStatus >> 0) & 0xff;

	//***********************************上面这1条表示设备是否允许绑定************************//

	//158	启用短信告警	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
	mqtt_upload_buffer[index++] = 158 & 0xff;
	mqtt_upload_buffer[index++] = 0x05;//bitmap
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlagSms >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlagSms >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlagSms >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlagSms >> 0) & 0xff;

	//100	警报开关	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
	mqtt_upload_buffer[index++] = 100 & 0xff;
	mqtt_upload_buffer[index++] = 0x05;//bitmap
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlag >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlag >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlag >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmFlag >> 0) & 0xff;

	//101	警报通知	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
	mqtt_upload_buffer[index++] = 101 & 0xff;
	mqtt_upload_buffer[index++] = 0x05;//bitmap
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_alarmStatus >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmStatus >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmStatus >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_alarmStatus >> 0) & 0xff;

	//102	电量	value	int32类型：0-100之间是当前电量，255表示插电无电池，当前电量加150表示正在充电（150-250之间）
	mqtt_upload_buffer[index++] = 102 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_battery >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_battery >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_battery >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_battery >> 0) & 0xff;

	if(upload_all_once){
		//103	圆形地理围栏（GCJ02）	Raw	12个字节一组（每组一个圆），可多组（多个圆）。每组数据：纬度(int32)+经度(int32)+半径（int32），经纬度都放大10^7倍，半径单位是厘米。
		//104	圆形地理围栏（WGS84）	Raw	12个字节一组（每组一个圆），可多组（多个圆）。每组数据：纬度(int32)+经度(int32)+半径（int32），经纬度都放大10^7倍，半径单位是厘米。
		if(dpValue_data_geofencing_length > 0){//有围栏数据的时候，上报围栏
			if(dpValue_gpsType == 0){
				mqtt_upload_buffer[index++] = 104 & 0xff;//圆形地理围栏（WGS84）
			}
			else if(dpValue_gpsType == 1){
				mqtt_upload_buffer[index++] = 103 & 0xff;//圆形地理围栏（GCJ02）
			}
			mqtt_upload_buffer[index++] = 0x00;//raw
			mqtt_upload_buffer[index++] = (dpValue_data_geofencing_length >> 8) & 0xff;
			mqtt_upload_buffer[index++] = dpValue_data_geofencing_length & 0xff;
			for (size_t i = 0; i < dpValue_data_geofencing_length; i++)
			{
				mqtt_upload_buffer[index++] = dpValue_data_geofencing[i];
			}
		}
		else{//没有围栏的时候，上报null
			if(dpValue_gpsType == 0){
				mqtt_upload_buffer[index++] = 104 & 0xff;//圆形地理围栏（WGS84）
			}
			else if(dpValue_gpsType == 1){
				mqtt_upload_buffer[index++] = 103 & 0xff;//圆形地理围栏（GCJ02）
			}
			mqtt_upload_buffer[index++] = 0x00;//raw
			mqtt_upload_buffer[index++] = 0x00;
			mqtt_upload_buffer[index++] = 0x00;//当长度等于0时，数据就是null
		}
	}
	
	//106	设备坐标格式	enum	0：WGS84 1：GCJ02
	mqtt_upload_buffer[index++] = 106 & 0xff;
	mqtt_upload_buffer[index++] = 0x04;//enum
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_gpsType >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_gpsType >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_gpsType >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_gpsType >> 0) & 0xff;

	//107	GPS经纬度（WGS84）	Raw	全部大端，前后排列：纬度(int32),经度(int32),海拔(int32),精度(int32)厘米【经纬度放大10^7倍，海拔单位厘米】
    if(gnss_lat != 255 && gnss_lon != 255 && dpValue_hasSatellite){

		int32_t latitude7 = gnss_lat * 10000000;//纬度
		int32_t longitude7 = gnss_lon * 10000000;//经度
		int32_t accuracy100 = gnss_accuracy * 100;//厘米

		mqtt_upload_buffer[index++] = 107 & 0xff;
		mqtt_upload_buffer[index++] = 0x00;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 16 & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 0) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 0) & 0xff;
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = (accuracy100 >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (accuracy100 >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (accuracy100 >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (accuracy100 >> 0) & 0xff;

	}

	//187	自带LBS定位（WGS84）	Raw	（当GPS没有时上报）全部大端，前后排列：纬度(int32),经度(int32),海拔(int32),精度(int32)厘米【经纬度放大10^7倍，海拔单位厘米】
	if(lbs_lat != 255 && lbs_lon != 255 && !dpValue_hasSatellite){

		int32_t latitude7 = lbs_lat * 10000000;//纬度
		int32_t longitude7 = lbs_lon * 10000000;//经度
		int32_t accuracy100 = lbs_accuracy * 100;//厘米

		mqtt_upload_buffer[index++] = 187 & 0xff;
		mqtt_upload_buffer[index++] = 0x00;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 16 & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (latitude7 >> 0) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (longitude7 >> 0) & 0xff;
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = 0x00;//海拔
		mqtt_upload_buffer[index++] = (accuracy100 >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (accuracy100 >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (accuracy100 >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (accuracy100 >> 0) & 0xff;

	}


	//108	有无卫星定位信号	bool	
	mqtt_upload_buffer[index++] = 108 & 0xff;
	mqtt_upload_buffer[index++] = 0x01;//bool
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x01;
	mqtt_upload_buffer[index++] = dpValue_hasSatellite ? 0x01 : 0x00;

	if(upload_all_once){
		//109	设备的能力（决定了app显示的内容）	bitmap	按bit位从低到高：智能定位|gps|北斗|LBS|WiFi|SOS电话|SOS实时语音|语音分段对讲|语音分段监听|语音实时监听
		mqtt_upload_buffer[index++] = 109 & 0xff;
		mqtt_upload_buffer[index++] = 0x05;//bitmap
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_functionFlag >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_functionFlag >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_functionFlag >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_functionFlag >> 0) & 0xff;
	}

	if(upload_all_once){
		//112	温度上限	value	（产品需要才用）放大100倍
		mqtt_upload_buffer[index++] = 112 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_tempMaxLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_tempMaxLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_tempMaxLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_tempMaxLimit >> 0) & 0xff;

		//113	温度下限	value	（产品需要才用）放大100倍
		mqtt_upload_buffer[index++] = 113 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_tempMinLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_tempMinLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_tempMinLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_tempMinLimit >> 0) & 0xff;

		//114	湿度上限	value	（产品需要才用）放大100倍
		mqtt_upload_buffer[index++] = 114 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_humMaxLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_humMaxLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_humMaxLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_humMaxLimit >> 0) & 0xff;

		//115	湿度下限	value	（产品需要才用）放大100倍
		mqtt_upload_buffer[index++] = 115 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_humMinLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_humMinLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_humMinLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_humMinLimit >> 0) & 0xff;
	}

	
	//116	温度	value	放大100倍，单位摄氏度 int32
	mqtt_upload_buffer[index++] = 116 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_currentTemp >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentTemp >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentTemp >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentTemp >> 0) & 0xff;

	//117	湿度	value	放大100倍 int32
	mqtt_upload_buffer[index++] = 117 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_currentHum >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentHum >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentHum >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentHum >> 0) & 0xff;

	//118	气压	value	不放大。int32。单位：帕斯卡
	mqtt_upload_buffer[index++] = 118 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_currentPascal >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentPascal >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentPascal >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_currentPascal >> 0) & 0xff;


	if(upload_all_once){
		//111	气压下限	value	（产品需要才用）不放大。int32。单位：帕斯卡
		mqtt_upload_buffer[index++] = 111 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_PascalMinLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_PascalMinLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_PascalMinLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_PascalMinLimit >> 0) & 0xff;


		//110	气压上限	value	（产品需要才用）不放大。int32。单位：帕斯卡
		mqtt_upload_buffer[index++] = 110 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_PascalMaxLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_PascalMaxLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_PascalMaxLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_PascalMaxLimit >> 0) & 0xff;
	}

	if(upload_all_once){
		//120	远程开关机	bool	关机就是最低功耗联网状态，支持实时远程开机
		mqtt_upload_buffer[index++] = 120 & 0xff;
		mqtt_upload_buffer[index++] = 0x01;//bool
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x01;
		mqtt_upload_buffer[index++] = dpValue_power_on ? 0x01 : 0x00;


		//121	定位频率	value	0：低功耗待机(不主动定位上报，可随时接收指令立即定位一下上报)；大于0：定位间隔时间（单位：秒）
		mqtt_upload_buffer[index++] = 121 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_frequency >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_frequency >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_frequency >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_frequency >> 0) & 0xff;
	}


	if(upload_all_once){

		//130	ICCID	string	

		//131	sim卡号	string	

		//132	sim手机号	string	

		//133	设备模组类型	string	可以是方案型号或模组型号
		mqtt_upload_buffer[index++] = 133 & 0xff;
		mqtt_upload_buffer[index++] = 0x03;
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = strlen(dpValue_model_type) & 0xff;

		for (size_t i = 0; i < strlen(dpValue_model_type); i++)
		{
			mqtt_upload_buffer[index++] = dpValue_model_type[i];
		}


		//134	蜂窝信号量	value	int32，例如： -82
		mqtt_upload_buffer[index++] = 134 & 0xff;
		mqtt_upload_buffer[index++] = 0x04;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_mobileSignal >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_mobileSignal >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_mobileSignal >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_mobileSignal >> 0) & 0xff;

		//135	蜂窝类型	enum	2：GPRS ；3：3G；4：4G
		mqtt_upload_buffer[index++] = 135 & 0xff;
		mqtt_upload_buffer[index++] = 0x04;//enum
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_mobileType >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_mobileType >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_mobileType >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_mobileType >> 0) & 0xff;

		//136	IMEI号码	string	IMEI号码
		ret = luat_mobile_get_imei(0, IMEI_STR, sizeof(IMEI_STR)-1);

		mqtt_upload_buffer[index++] = 136 & 0xff;
		mqtt_upload_buffer[index++] = 0x03;
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = strlen(IMEI_STR) & 0xff;

		for (size_t i = 0; i < strlen(IMEI_STR); i++)
		{
			mqtt_upload_buffer[index++] = IMEI_STR[i];
		}

	}


	//140	静止时长	value	单位：秒（正数是静止时长，负数是运动时长）
	mqtt_upload_buffer[index++] = 140 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_stopTime >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_stopTime >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_stopTime >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_stopTime >> 0) & 0xff;

	//141	速度	value	米，int32
	mqtt_upload_buffer[index++] = 141 & 0xff;
	mqtt_upload_buffer[index++] = 0x02;//value
	mqtt_upload_buffer[index++] = 0x00;
	mqtt_upload_buffer[index++] = 0x04;
	mqtt_upload_buffer[index++] = (dpValue_speed >> 24) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_speed >> 16) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_speed >> 8) & 0xff;
	mqtt_upload_buffer[index++] = (dpValue_speed >> 0) & 0xff;

	if(upload_all_once){
		//142	速度限制	value	米，int32，超速后需警报（负数无限制）
		mqtt_upload_buffer[index++] = 142 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_speedLimit >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_speedLimit >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_speedLimit >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_speedLimit >> 0) & 0xff;
	}

	if(upload_all_once){
		//200	固件版本	value	int32，自然数 正数
		mqtt_upload_buffer[index++] = 200 & 0xff;
		mqtt_upload_buffer[index++] = 0x02;//value
		mqtt_upload_buffer[index++] = 0x00;
		mqtt_upload_buffer[index++] = 0x04;
		mqtt_upload_buffer[index++] = (dpValue_version >> 24) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_version >> 16) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_version >> 8) & 0xff;
		mqtt_upload_buffer[index++] = (dpValue_version >> 0) & 0xff;
	}

	//payload length
	uint16_t payload_length = index-5;

	mqtt_upload_buffer[3] = (payload_length >> 8) & 0xff;
	mqtt_upload_buffer[4] = (payload_length >> 0) & 0xff;

	//CRC
	uint16_t crc_result = CRC16_CCITT_FALSE(&mqtt_upload_buffer[2], payload_length+3);
	uint8_t crc_result_1 = (crc_result >> 8) & 0xff;
	uint8_t crc_result_2 = crc_result & 0xff;

	mqtt_upload_buffer[payload_length+5+0] = crc_result_1;
	mqtt_upload_buffer[payload_length+5+1] = crc_result_2;

	uint16_t length_send = payload_length+7;

	//MQTT
	uint16_t message_id = 0;
	mqtt_publish_with_qos(&(luat_mqtt_ctrl->broker), mqtt_pub_topic, mqtt_upload_buffer, length_send, 0, MQTT_DEMO_PUB_QOS, &message_id);

	LUAT_DEBUG_PRINT("done");

	upload_all_once = false;//改回去，下次当true时，才会全部重新上传，一般情况下只上传最少必要数据

	

}

void luat_mqtt_task(void *param)
{
	int ret = -1;

	uint32_t power_on_seconds = 0;//开机了多长时间（单位：100ms）

	uint32_t upload_seconds = 0;//距离上一次上传过去了多少时间（单位：100ms）

	uint32_t upload_interval = 12000;//每次至少上报间隔20分钟（单位：100ms）太长了会显示离线，太短了耗电

	//等待HTTP任务从平台获取MQTT账号
	while (strlen(mqttUser) == 0 || strlen(mqttPwd) == 0 || strlen(mqttDeviceId) == 0){
		LUAT_DEBUG_PRINT("waiting for mqtt account init...");
		luat_rtos_task_sleep(1000);
	}


	//组装好MQTT的主题
	//上报主题 up
	int topic_up_len = strlen(topic_up_prex) + strlen(mqttDeviceId) + 1; // 计算拼接后的字符串长度（加1是为了空出字符串末尾的'\0'）
	mqtt_pub_topic = (char *)malloc(topic_up_len + 1); // 为拼接后的字符串分配内存空间（加1也是为了空出字符串末尾的'\0'）

	memset(mqtt_pub_topic,0x00,topic_up_len + 1);

	strcpy(mqtt_pub_topic, topic_up_prex); // 将topic_up_prex拷贝到topic_up中
	strcat(mqtt_pub_topic, mqttDeviceId); // 将device_id拼接到topic_up后面

	int topic_down_len = strlen(topic_down_prex) + strlen(mqttDeviceId) + 1;
	mqtt_sub_topic = (char *)malloc(topic_down_len + 1); 

	memset(mqtt_sub_topic,0x00,topic_down_len + 1);

	strcpy(mqtt_sub_topic, topic_down_prex); 
	strcat(mqtt_sub_topic, mqttDeviceId);

	// 组装完毕

	LUAT_DEBUG_PRINT("mqtt_pub_topic %s", mqtt_pub_topic);
	LUAT_DEBUG_PRINT("mqtt_sub_topic %s", mqtt_sub_topic);


	luat_mqtt_ctrl_t *luat_mqtt_ctrl = (luat_mqtt_ctrl_t *)luat_heap_malloc(sizeof(luat_mqtt_ctrl_t));
	ret = luat_mqtt_init(luat_mqtt_ctrl, NW_ADAPTER_INDEX_LWIP_GPRS);
	if (ret) {
		LUAT_DEBUG_PRINT("mqtt init FAID ret %d", ret);
		return;
	}
	luat_mqtt_ctrl->ip_addr.type = 0xff;
	luat_mqtt_connopts_t opts = {0};

	opts.is_tls = 0; 
	opts.host = MQTT_HOST;
	opts.port = MQTT_PORT;
	ret = luat_mqtt_set_connopts(luat_mqtt_ctrl, &opts);

	char clientId[16] = {0};
	ret = luat_mobile_get_imei(0, clientId, sizeof(clientId)-1);
	if(ret <= 0){
		LUAT_DEBUG_PRINT("imei get fail");
		mqtt_init(&(luat_mqtt_ctrl->broker), mqttDeviceId);
	}
	else
		mqtt_init(&(luat_mqtt_ctrl->broker), clientId);

	mqtt_init_auth(&(luat_mqtt_ctrl->broker), mqttUser, mqttPwd);

	// luat_mqtt_ctrl->netc->is_debug = 1;// debug信息
	luat_mqtt_ctrl->broker.clean_session = 1;
	luat_mqtt_ctrl->keepalive = 240;

if (MQTT_DEMO_AUTOCON == 1)
{
	luat_mqtt_ctrl->reconnect = 1;
	luat_mqtt_ctrl->reconnect_time = 3000;
}

	luat_mqtt_set_will(luat_mqtt_ctrl, mqtt_pub_topic, mqtt_will_payload, strlen(mqtt_will_payload), 0, 0); // 测试遗嘱
	
	luat_mqtt_set_cb(luat_mqtt_ctrl,luat_mqtt_cb);
	luat_mqtt_ctrl->netc->is_debug = 1;
	LUAT_DEBUG_PRINT("mqtt_connect");
	ret = luat_mqtt_connect(luat_mqtt_ctrl);
	if (ret) {
		LUAT_DEBUG_PRINT("mqtt connect ret=%d\n", ret);
		luat_mqtt_close_socket(luat_mqtt_ctrl);
		return;
	}
	LUAT_DEBUG_PRINT("wait mqtt_state ...");

	while(1){

		power_on_seconds++;
		upload_seconds++;

		//至少每upload_interval个100毫秒上报一次，或者当check_and_upload_once==1时立即上报
		if(check_and_upload_once == 1 || (upload_seconds >= upload_interval)){//（单位：100ms）
			if (luat_mqtt_state_get(luat_mqtt_ctrl) == MQTT_STATE_READY){
                check_and_upload_once = 0;
                checkAll();
			    uploadMessage(luat_mqtt_ctrl);
				upload_seconds = 0;//归零重新计算时间
            }
		}

		luat_rtos_task_sleep(100);
		
		//开机2分钟后禁止被绑定
		if(power_on_seconds == 1200){
			pairStatus = 0;
			check_and_upload_once = 1;
		}

	}
}