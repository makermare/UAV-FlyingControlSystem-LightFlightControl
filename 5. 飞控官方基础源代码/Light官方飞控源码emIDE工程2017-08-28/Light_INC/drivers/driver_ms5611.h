/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_ms5611.h
 + 描述    ：气压计驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_MS5611_H
#define _DRIVER_MS5611_H

#include "stm32f4xx.h"
#include "device_iic.h"
#include "device_ms5611.h"
#include "driver_mpu6050.h"
#include "database.h"

/* 气压计硬件故障 */
extern u8 hard_error_ms5611;
/* 气压计计算高度，单位mm(毫米) */
extern int32_t baroAlt,baroAltOld;
/* 气压计计算速度单位mm/s */
extern float baro_alt_speed;

/*----------------------------------------------------------
 + 实现功能：气压计初始化
----------------------------------------------------------*/
extern void MS5611_Init(void);
/*----------------------------------------------------------
 + 实现功能：气压计数据更新 由任务调度调用周期10ms
----------------------------------------------------------*/
extern int MS5611_Update(void);
/*----------------------------------------------------------
 + 实现功能：气压计高度计算
----------------------------------------------------------*/
extern void MS5611_BaroAltCalculate(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
