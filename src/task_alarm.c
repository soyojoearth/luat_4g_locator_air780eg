#include <stdio.h>
#include <math.h>
#include "luat_network_adapter.h"
#include "common_api.h"
#include "luat_debug.h"
#include "luat_mem.h"
#include "luat_rtos.h"
#include "luat_mobile.h"

#include "global.h"

#include "task_alarm.h"

#include "task_lbs.h"

luat_rtos_task_handle alarm_task_handle;


bool isOutGeofencing = false;//默认当前没有越界

uint8_t checkIsOutGeofencing(){

    //检查电子围栏，是否越界
    // uint8_t dpValue_data_geofencing[252] = {0};//12个字节1个围栏，一共可以存21个围栏
    // int16_t dpValue_data_geofencing_length = 0;

    int32_t distanceLimit = 0;
    double distanceLimitDouble = 0;

    int32_t latInt = 2410000000;
    int32_t lonInt = 2410000000;

    double latDouble = 2410000000;
    double lonDouble = 2410000000;

    
    // LUAT_DEBUG_PRINT("Location lbs_lat: %0.7f lbs_lon: %0.7f \n", lbs_lat, lbs_lon);


    if (dpValue_data_geofencing_length == 0)
    {
        // LUAT_DEBUG_PRINT("Geofencing empty");
        return 0;//没有越界
    }
    // else{
    //     // LUAT_DEBUG_PRINT("Geofencing :%i", dpValue_data_geofencing_length);
    // }
    

    int k = 0;
    for (size_t i = 0; i < dpValue_data_geofencing_length; i+=12)
    {

        latInt = ((dpValue_data_geofencing[i+0] & 0xffffffff) << 24) | 
					((dpValue_data_geofencing[i+1] & 0xffffffff) << 16) | 
					((dpValue_data_geofencing[i+2] & 0xffffffff) << 8) | 
					((dpValue_data_geofencing[i+3] & 0xffffffff) << 0);

                    latDouble = (double)latInt / (double)10000000;

        lonInt = ((dpValue_data_geofencing[i+4] & 0xffffffff) << 24) | 
					((dpValue_data_geofencing[i+5] & 0xffffffff) << 16) | 
					((dpValue_data_geofencing[i+6] & 0xffffffff) << 8) | 
					((dpValue_data_geofencing[i+7] & 0xffffffff) << 0);

                    lonDouble = (double)lonInt / (double)10000000;


        distanceLimit = ((dpValue_data_geofencing[i+8] & 0xffffffff) << 24) | 
					((dpValue_data_geofencing[i+9] & 0xffffffff) << 16) | 
					((dpValue_data_geofencing[i+10] & 0xffffffff) << 8) | 
					((dpValue_data_geofencing[i+11] & 0xffffffff) << 0);

        distanceLimitDouble = (double)distanceLimit / (double)100000;

        // LUAT_DEBUG_PRINT("Geofencing %i, latDouble = %0.7f lonDouble = %0.7f\n distanceLimit = %0.7f", k, latDouble, lonDouble, distanceLimitDouble);

        k++;

        // lbs_lat;//纬度
        // lbs_lon;//经度
        
        double dst = get_distance(lbs_lat, lbs_lon, latDouble, lonDouble);

        // LUAT_DEBUG_PRINT("dst = %0.3fkm\n", dst);  //dst = 9.281km

        if (dst < distanceLimitDouble || distanceLimitDouble == 0)
        {
            //只要在其中任何一个围栏之内，就没有越界
            return 0;
        }
                
    }

    return 1;//不在任何一个围栏内，越界了
    
}

/**
 * 陀螺仪实时速度（单位：米每秒）
*/
int32_t acceleration_speed_current(){

    //这里计算瞬时速度

    //todo

    return 0;

}

/**
 * 陀螺仪实时加速度
*/
double acceleration_current(){


    //这里计算加速度

    //todo


    
    return 0;

}

void checkAll(){

    int8_t res = 0;

    if(lbs_lat != 255 && lbs_lon != 255){
        
        //检查是否围栏越界
        res = checkIsOutGeofencing();

        if (res)
        {
            // LUAT_DEBUG_PRINT("checkIsOutGeofencing:YES\n");
            if (!isOutGeofencing)//是不是刚越界
            {
                //越界了
                LUAT_DEBUG_PRINT("OutGeofencing First\n");
                isOutGeofencing = true;
                if(dpValue_alarmFlag & (1 << 3)){//通知开关是否打开
                    LUAT_DEBUG_PRINT("Alarm 3\n");
                    dpValue_alarmStatus |= (1 << 3);//出围栏置1
                }
                if(dpValue_alarmFlag & (1 << 2)){//通知开关是否打开
                    LUAT_DEBUG_PRINT("Alarm 2\n");
                    dpValue_alarmStatus &= ~(1 << 2);//进围栏置0
                }
            }
        }
        else{
            // LUAT_DEBUG_PRINT("checkIsOutGeofencing:NO\n");
            if (isOutGeofencing)//是不是刚回来
            {
                //回界了
                LUAT_DEBUG_PRINT("BackGeofencing First\n");
                isOutGeofencing = false;
                if(dpValue_alarmFlag & (1 << 3)){//通知开关是否打开
                    LUAT_DEBUG_PRINT("Alarm 3\n");
                    dpValue_alarmStatus &= ~(1 << 3);//出围栏置0
                }
                if(dpValue_alarmFlag & (1 << 2)){//通知开关是否打开
                    LUAT_DEBUG_PRINT("Alarm 2\n");
                    dpValue_alarmStatus |= (1 << 2);//进围栏置1
                }
            }
        }

    }

    //检查温湿度

    //检查其它。。。




    /****
     * 判断是不是静止状态（相对地球是不是静止）
     * 物体不存在绝对静止状态，只有相对静止状态。加速度计输出是0的时候，无法判断是相对地球静止，还是相对匀速运动的车、船静止。
     * 判断条件：
     * 在gps变化的时候，那么相对地球一定没有静止；
     * 在加速度不等于0的时候，那么相对任何东西都没有静止。
     * 所以，只有当加速度等于0，且GPS也没有距离变化时，才是相对地球静止。
     * */    
    if(acceleration_current() != 0)
    {
        LUAT_DEBUG_PRINT("is Move by ACC\n");
        //在动
        if(dpValue_stopTime > 0){
            dpValue_stopTime--;//计算静止时长 单位：秒（正数是静止时长，负数是运动时长）
        }
        else{
            dpValue_stopTime = -1;
        }
    }
    else{

        if(lat_last != 255 && lon_last != 255 && lat_current != 255 && lon_current != 255 && dpValue_frequency > 0){//有GPS定位模式){
            //距离上一次超过10米，算移动（精度原因）
            if(get_distance(lat_last, lon_last, lat_current, lon_current) > 0.01){
                LUAT_DEBUG_PRINT("is Move by GPS\n");
                //在动
                if(dpValue_stopTime < 0){
                    dpValue_stopTime--;//计算静止时长 单位：秒（正数是静止时长，负数是运动时长）
                }
                else{
                    dpValue_stopTime = -1;
                }
            }
            else{
                LUAT_DEBUG_PRINT("not Move by GPS\n");
                //静止
                if(dpValue_stopTime > 0){
                    dpValue_stopTime++;//计算静止时长 单位：秒（正数是静止时长，负数是运动时长）
                }
                else{
                    dpValue_stopTime = 1;
                }
            }
        }
        else{
            LUAT_DEBUG_PRINT("not Move on Begin\n");
            //没有加速度，也还没有gps的时候，默认静止
            if(dpValue_stopTime > 0){
                dpValue_stopTime++;//计算静止时长 单位：秒（正数是静止时长，负数是运动时长）
            }
            else{
                dpValue_stopTime = 1;
            }
        }

    }

    /**
     * 单纯靠加速度来算速度，不一定可靠，因为匀速运动时加速度是0；要结合gps和加速度计，一起算速度才行。
     * 先要依靠gps来算出当前参考系(车、船)相对于地球的移动速度，然后把加速度计的速度加上去，才是设备相对于地球的运动速度
    */
    if(lat_last != 255 && lon_last != 255 && lat_current != 255 && lon_current != 255 && dpValue_frequency > 0){
        dpValue_speed = get_distance(lat_last, lon_last, lat_current, lon_current) * 1000 / dpValue_frequency;
        dpValue_speed += acceleration_speed_current();
        LUAT_DEBUG_PRINT("Speed GPS+ACC:%i\n",dpValue_speed);
    }
    else{
        //没有gps的时候，只剩下室内的移动速度
        dpValue_speed = acceleration_speed_current();
        LUAT_DEBUG_PRINT("Speed ACC:%i\n",dpValue_speed);
    }
    

    //超速告警
    if(dpValue_speed > dpValue_speedLimit){
        if(dpValue_alarmFlag & (1 << 5)){//通知开关是否打开
            LUAT_DEBUG_PRINT("Alarm 5\n");
            //超速告警
            dpValue_alarmStatus |= (1 << 5);
        }
    }
    else{
        //不超速了
        dpValue_alarmStatus &= ~(1 << 5);
    }
}

void luat_alarm_task(void *param)
{
    while(1){

		//检测任务
        checkAll();

        //其它。。。


		luat_rtos_task_sleep(1000);

	}
}