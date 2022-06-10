/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：position_control.h
 + 描述    ：GPS位置控制头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef __POSITION_CONTROL_H__
#define __POSITION_CONTROL_H__

#include "stm32f4xx.h"
#include "driver_GPS.h"

/* 悬停经纬度 坐标 单位 放大率10E5 */
extern int STOP_longitude,STOP_latitude;

/* 速度转换期望角度 */
extern float expect_angle_pitch,expect_angle_roll;

/* 当前位置 单位 放大率10E5 */
extern int t_longitude,t_latitude;

/* 方位 速度 放大率10E3 单位 毫米每秒 */
extern int speed_longitude,speed_latitude;

/* 飞行器 速度 单位毫米每秒 */
extern float Speed_Front,Speed_Left;

/*----------------------------------------------------------
 + 实现功能：串级PID控制悬停
 + 调用参数：两次调用时间间隔
----------------------------------------------------------*/
extern void PositionControl_Mode(float Timer_t);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
