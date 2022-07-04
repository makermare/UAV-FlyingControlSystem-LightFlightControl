/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：device_led.h
 + 描述    ：led设备头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DEVICE_LED_H_
#define	_DEVICE_LED_H_

#include "stm32f4xx.h"

/* LED 电平宏定义 */
#define LED1_OFF         GPIO_LED->BSRRL = Pin_LED1
#define LED1_ON          GPIO_LED->BSRRH = Pin_LED1
#define LED2_OFF         GPIO_LED->BSRRL = Pin_LED2
#define LED2_ON          GPIO_LED->BSRRH = Pin_LED2
#define LED3_OFF         GPIO_LED->BSRRL = Pin_LED3
#define LED3_ON          GPIO_LED->BSRRH = Pin_LED3
#define LED4_OFF         GPIO_LED->BSRRL = Pin_LED4
#define LED4_ON          GPIO_LED->BSRRH = Pin_LED4

/* LED 的GPIO宏定义 */
#define RCC_LED			RCC_AHB1Periph_GPIOE
#define GPIO_LED		GPIOE
#define Pin_LED1		GPIO_Pin_3
#define Pin_LED2		GPIO_Pin_2
#define Pin_LED3		GPIO_Pin_1
#define Pin_LED4		GPIO_Pin_0

/*----------------------------------------------------------
 + 实现功能：控制LED初始化
----------------------------------------------------------*/
extern void LED_Init(void);

/*----------------------------------------------------------
 + 实现功能：控制LED发光亮度 由任务调度调用周期1ms
 + 调用参数功能：发光亮度数组 0-20
----------------------------------------------------------*/
extern void Call_LED_show(u8 duty[4]);

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
