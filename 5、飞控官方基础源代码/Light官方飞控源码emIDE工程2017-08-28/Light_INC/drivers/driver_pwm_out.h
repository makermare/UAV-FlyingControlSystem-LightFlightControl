/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  driver_pwm_out.h
 + 描述    ：PWM输出驱动头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DRIVER_PWM_OUT_H_
#define _DRIVER_PWM_OUT_H_

#include "stm32f4xx.h"
#include "device_pwm_out.h"

/* PWM的8个通道输出模式 */
extern int PWM_Mode;

/*----------------------------------------------------------
 + 实现功能：PWM的8个通道输出数据的调用
 + 调用参数功能：双字节整数数组：设置的数据(0-1000)
----------------------------------------------------------*/
extern void SetPwm(int16_t set_8pwm[]);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
