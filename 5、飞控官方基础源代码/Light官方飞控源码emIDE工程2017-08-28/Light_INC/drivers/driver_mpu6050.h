/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：driver_mpu6050.h
 + 描述    ：IMU传感器mpu6050驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_MPU6050_H
#define _DRIVER_MPU6050_H

#include "stm32f4xx.h"
#include "database.h"
#include "device_mpu6050.h"

/* MPU6050结构体 */
extern MPU6050_STRUCT mpu6050;

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期2ms
----------------------------------------------------------*/
extern void Call_MPU6050_Data_Prepare(float T);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
