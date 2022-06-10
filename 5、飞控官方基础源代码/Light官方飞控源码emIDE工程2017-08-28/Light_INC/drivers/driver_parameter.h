/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_parameter.h
 + 描述    ：默认参数及校准数据文件头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_PARAMETER_H
#define	_DRIVER_PARAMETER_H

#include "stm32f4xx.h"
#include "database.h"

/* PID参数数组 */
pid_setup_t pid_setup;

/*----------------------------------------------------------
 + 实现功能：PID参数恢复默认
----------------------------------------------------------*/
extern void Para_ResetToFactorySetup(void);

/*----------------------------------------------------------
 + 实现功能：所有参数初始化
----------------------------------------------------------*/
extern void Para_Init(void);

/*----------------------------------------------------------
 + 实现功能：保存加速度计的校准参数
----------------------------------------------------------*/
extern void Param_SaveAccelOffset(xyz_f_t *offset);

/*----------------------------------------------------------
 + 实现功能：保存陀螺仪的校准参数
----------------------------------------------------------*/
extern void Param_SaveGyroOffset(xyz_f_t *offset);

/*----------------------------------------------------------
 + 实现功能：保存磁力计的校准参数
----------------------------------------------------------*/
extern void Param_SaveMagOffset(xyz_f_t *offset);

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期20ms
----------------------------------------------------------*/
extern void Parameter_Save(void);

/*----------------------------------------------------------
 + 实现功能：所有参数初始化
----------------------------------------------------------*/
extern void PID_Para_Init(void);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
