/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：driver_led.h
 + 描述    ：led指示灯状态控制头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_LED_H_
#define	_DRIVER_LED_H_

#include "stm32f4xx.h"
#include "device_led.h"

/* 每个LED亮度范围是0-20 */
extern u8 LED_Brightness[4];

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期50ms
----------------------------------------------------------*/
extern void Call_LED_duty(void);

/*----------------------------------------------------------
 + 实现功能：MPU故障指示
----------------------------------------------------------*/
extern void LED_MPU_Err(void);

/*----------------------------------------------------------
 + 实现功能：磁力计故障指示
----------------------------------------------------------*/
extern void LED_Mag_Err(void);

/*----------------------------------------------------------
 + 实现功能：气压计故障指示
----------------------------------------------------------*/
extern void LED_MS5611_Err(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
