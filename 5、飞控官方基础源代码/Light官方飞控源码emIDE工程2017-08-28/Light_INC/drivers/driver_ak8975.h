/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：driver_ak8975.h
 + 描述    ：磁力计（电子罗盘）驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_AK8975_H_
#define	_DRIVER_AK8975_H_

#include "stm32f4xx.h"
#include "database.h"

/* 磁力计数组：采样值,偏移值,纠正后的值 */
extern ak8975_t ak8975;
/* 磁力计硬件故障 */
extern u8 hard_error_ak8975;
/* 磁力计校准标识 */
extern u8 Mag_CALIBRATED;

/*----------------------------------------------------------
 + 实现功能：磁力计采样触发
 + 返回值：磁力计运行状态
----------------------------------------------------------*/
extern uint8_t AK8975_IS_EXIST(void);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期10ms
----------------------------------------------------------*/
extern void Call_AK8975(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
