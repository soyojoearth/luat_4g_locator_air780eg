#include <stdio.h>
#include <math.h>

#include "common_api.h"
#include "luat_rtos.h"
#include "luat_mobile.h"
#include "luat_network_adapter.h"
#include "luat_debug.h"
#include "luat_mobile.h"
#include "net_lwip.h"

#include "global.h"

#define PI                      3.1415926
#define EARTH_RADIUS            6378.137        //地球近似半径


//这里填写平台分配给您的 产测账号、密码
char* facId = "585659";
char* facSecret = "abc123abc";
// char* facId = "";
// char* facSecret = "";
//一定要有产测账号，才能自动获取到MQTT授权码，请向我们索取！微信: napshen （申工）
//上面写在里面的产测账号的密码已经改掉了（之前不小心上次到了github），请另外向我们索取

//flag
int8_t check_and_upload_once = 1;//立即准备后续处理并上报数据
bool upload_all_once = true;//是否全部数据上传一次，为了节省流量，仅在必要的时候上传一次全部数据，比如刚联网、下发指令重新上传等



//这里填写模组类型
char* clientType = "Air780EP";

//填写平台上的ProductId
char* product_id = "iR3kDi9B8Qd";

//133	设备模组类型	string	可以是方案型号或模组型号
char* dpValue_model_type = "air780epv";

//158	启用短信告警	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
int32_t dpValue_alarmFlagSms = 0;//启用短信告警

//100	警报开关	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
int32_t dpValue_alarmFlag = 0;//警报开关

//101	警报通知	bitmap	按bit位低到高：故障警报|低电量警报|进围栏警报|出围栏警报|震动警报|超速警报|防拆警报|落水警报|温度警报|湿度警报|气压警报|手动警报|移动侦测
int32_t dpValue_alarmStatus = 0;//警报通知	

bool dpValue_power_on = true;//远程开关机

//121	定位频率	value	0：低功耗待机(不主动定位上报，可随时接收指令立即定位一下上报)；大于0：定位间隔时间（单位：秒）
int32_t dpValue_frequency = 180;//定位频率

int32_t dpValue_mobileSignal = -80;//蜂窝信号
//102	电量	value	int32类型：0-100之间是当前电量，255表示插电无电池，当前电量加150表示正在充电（150-250之间）
int32_t dpValue_battery = 98;//模拟数据
//200	固件版本	value	int32，自然数 正数
int32_t dpValue_version = 2;//模拟数据
//140	静止时长	value	单位：秒（正数是静止时长，负数是运动时长）
int32_t dpValue_stopTime = 1;//模拟数据
//135	蜂窝类型	enum	2：GPRS ；3：3G；4：4G
int32_t dpValue_mobileType = 4;//4G
//141	速度	value	米，int32
int32_t dpValue_speed = 0;//模拟数据

//142	速度限制	value	米，int32，超速后需警报（负数无限制）
int32_t dpValue_speedLimit = 100000;


//109	设备的能力（决定了app显示的内容）	bitmap	按bit位从低到高：智能定位|gps|北斗|LBS|WiFi|SOS电话|SOS实时语音|语音分段对讲|语音分段监听|语音实时监听
int32_t dpValue_functionFlag = 255;
//108	有无卫星定位信号	bool	
bool dpValue_hasSatellite = false;


//106	设备坐标格式	enum	0：WGS84 1：GCJ02
int8_t dpValue_gpsType = 0;
//104	圆形地理围栏（WGS84）	Raw	12个字节一组（每组一个圆），可多组（多个圆）。每组数据：纬度(int32)+经度(int32)+半径（int32），经纬度都放大10^7倍，半径单位是厘米。
uint8_t dpValue_data_geofencing[252] = {0};//12个字节1个围栏，一共可以存21个围栏
int16_t dpValue_data_geofencing_length = 0;


//112	温度上限	value	（产品需要才用）放大100倍
int32_t dpValue_tempMaxLimit = 4000;//模拟数据

//113	温度下限	value	（产品需要才用）放大100倍
int32_t dpValue_tempMinLimit = 500;//模拟数据

//114	湿度上限	value	（产品需要才用）放大100倍
int32_t dpValue_humMaxLimit = 9000;//模拟数据

//115	湿度下限	value	（产品需要才用）放大100倍
int32_t dpValue_humMinLimit = 2000;//模拟数据

//116	温度	value	放大100倍，单位摄氏度 int32
int32_t dpValue_currentTemp = 2800;//模拟数据

//117	湿度	value	放大100倍 int32
int32_t dpValue_currentHum = 6500;//模拟数据


//118	气压	value	不放大。int32。单位：帕斯卡
int32_t dpValue_currentPascal= 1003;//模拟数据


//111	气压下限	value	（产品需要才用）不放大。int32。单位：帕斯卡
int32_t dpValue_PascalMinLimit = 800;


//110	气压上限	value	（产品需要才用）不放大。int32。单位：帕斯卡
int32_t dpValue_PascalMaxLimit = 1500;






double lat_last = 255;
double lon_last = 255;//上一次定位，用做计算有没有静止、移动的临时存储变量

double lat_current = 255;
double lon_current = 255;//当前定位，用做计算有没有静止、移动的临时存储变量


// 求弧度
double radian(double d)
{
    return d * PI / 180.0;   //角度1˚ = π / 180
}

//计算距离
double get_distance(double lat1, double lng1, double lat2, double lng2)
{
    double radLat1 = radian(lat1);
    double radLat2 = radian(lat2);
    double a = radLat1 - radLat2;
    double b = radian(lng1) - radian(lng2);
    
    double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2) )));
    
    dst = dst * EARTH_RADIUS;
    dst= round(dst * 10000) / 10000;
    return dst;
}