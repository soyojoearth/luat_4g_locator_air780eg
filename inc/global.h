#ifndef _GLOBAL_H
#define _GLOBAL_H

#include "common_api.h"
#include "luat_rtos.h"
#include "luat_mobile.h"
#include "luat_network_adapter.h"
#include "luat_debug.h"
#include "luat_mobile.h"
#include "net_lwip.h"

extern char* facId;
extern char* facSecret;

extern char* clientType;

extern char* product_id;

//flag
extern int8_t check_and_upload_once;//立即准备后续处理并上报数据
extern bool upload_all_once;//是否全部数据上传一次，为了节省流量，仅在必要的时候上传一次全部数据，比如刚联网、下发指令重新上传等



//数据

//158	启用短信告警	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
extern int32_t dpValue_alarmFlagSms;//启用短信告警

//100	警报开关	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
extern int32_t dpValue_alarmFlag;//警报开关	

//101	警报通知	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
extern int32_t dpValue_alarmStatus;//警报通知	

extern bool dpValue_power_on;//远程开关机

//121	定位频率	value	0：低功耗待机(不主动定位上报，可随时接收指令立即定位一下上报)；大于0：定位间隔时间（单位：秒）
extern int32_t dpValue_frequency;//定位频率

extern int32_t dpValue_mobileSignal;//蜂窝信号
//102	电量	value	int32类型：0-100之间是当前电量，255表示插电无电池，当前电量加150表示正在充电（150-250之间）
extern int32_t dpValue_battery;//模拟数据
//200	固件版本	value	int32，自然数 正数
extern int32_t dpValue_version;//模拟数据
//140	静止时长	value	单位：秒（正数是静止时长，负数是运动时长）
extern int32_t dpValue_stopTime;//模拟数据
//135	蜂窝类型	enum	2：GPRS ；3：3G；4：4G
extern int32_t dpValue_mobileType;//4G
//133	设备模组类型	string	可以是方案型号或模组型号
extern char* dpValue_model_type;

//141	速度	value	米，int32
extern int32_t dpValue_speed;//模拟数据

//142	速度限制	value	米，int32，超速后需警报（负数无限制）
extern int32_t dpValue_speedLimit;//模拟数据

//116	温度	value	放大100倍，单位摄氏度 int32
extern int32_t dpValue_currentTemp;//模拟数据
//109	设备的能力（决定了app显示的内容）	bitmap	按bit位从低到高：智能定位|gps|北斗|LBS|WiFi|SOS电话|SOS实时语音|语音分段对讲|语音分段监听|语音实时监听
extern int32_t dpValue_functionFlag;//模拟数据
//108	有无卫星定位信号	bool	
extern bool dpValue_hasSatellite;//模拟数据


//106	设备坐标格式	enum	0：WGS84 1：GCJ02
extern int8_t dpValue_gpsType;
//104	圆形地理围栏（WGS84）	Raw	12个字节一组（每组一个圆），可多组（多个圆）。每组数据：纬度(int32)+经度(int32)+半径（int32），经纬度都放大10^7倍，半径单位是厘米。
extern uint8_t dpValue_data_geofencing[252];//12个字节1个围栏，一共可以存21个围栏
extern int16_t dpValue_data_geofencing_length;


//112	温度上限	value	（产品需要才用）放大100倍
extern int32_t dpValue_tempMaxLimit;//模拟数据

//113	温度下限	value	（产品需要才用）放大100倍
extern int32_t dpValue_tempMinLimit;//模拟数据

//114	湿度上限	value	（产品需要才用）放大100倍
extern int32_t dpValue_humMaxLimit;//模拟数据

//115	湿度下限	value	（产品需要才用）放大100倍
extern int32_t dpValue_humMinLimit;//模拟数据

//116	温度	value	放大100倍，单位摄氏度 int32
extern int32_t dpValue_currentTemp;//模拟数据

//117	湿度	value	放大100倍 int32
extern int32_t dpValue_currentHum;//模拟数据

//118	气压	value	不放大。int32。单位：帕斯卡
extern int32_t dpValue_currentPascal;//模拟数据


//111	气压下限	value	（产品需要才用）不放大。int32。单位：帕斯卡
extern int32_t dpValue_PascalMinLimit;


//110	气压上限	value	（产品需要才用）不放大。int32。单位：帕斯卡
extern int32_t dpValue_PascalMaxLimit;





extern double lat_last;
extern double lon_last;//上一次定位，用做计算有没有静止、移动的临时存储变量

extern double lat_current;
extern double lon_current;//当前定位，用做计算有没有静止、移动的临时存储变量

//计算距离
double get_distance(double lat1, double lng1, double lat2, double lng2);

#endif