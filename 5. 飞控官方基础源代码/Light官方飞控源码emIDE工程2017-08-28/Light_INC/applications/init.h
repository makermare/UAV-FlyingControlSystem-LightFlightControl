/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：init.h
 + 描述    ：飞控初始化头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _INIT_H_
#define _INIT_H_

#include "stm32f4xx.h"

/* 初始化结束标识 */
extern u8 Init_Finish;

/*----------------------------------------------------------
 + 实现功能：飞控初始化
----------------------------------------------------------*/
extern void Light_Init(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
