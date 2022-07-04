/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_ultrasonic.h
 + 描述    ：超声波测距驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_ULTRASONIC_H
#define _DRIVER_ULTRASONIC_H

#include "stm32f4xx.h"
#include "database.h"

/* 超声波模块状态标志位：1准备接收高位 2准备接受低位 */
extern s8 ultra_state;
/* 超声波模块测量距离：单位毫米 */
extern u16 ultra_distance;
/* 超声波模块测量距离两次差：单位毫米 */
extern s16 ultra_delta;

/*----------------------------------------------------------
 + 实现功能：超声波模块初始化
----------------------------------------------------------*/
void Ultrasonic_Init();

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期100ms
----------------------------------------------------------*/
void Call_Ultrasonic();

/*----------------------------------------------------------
 + 实现功能：串口5接收到数据解析
 + 调用参数功能：单字节整数：串口接收到的数据
----------------------------------------------------------*/
void Ultra_Get(u8 com_data);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
