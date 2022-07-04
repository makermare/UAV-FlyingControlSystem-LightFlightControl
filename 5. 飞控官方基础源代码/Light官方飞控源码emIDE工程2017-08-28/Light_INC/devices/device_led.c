/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：device_led.c
 + 描述    ：led设备
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "device_led.h"
#include "stm32f4xx.h"

/*----------------------------------------------------------
 + 实现功能：控制LED初始化
----------------------------------------------------------*/
void LED_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_LED,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Pin   = Pin_LED1| Pin_LED2| Pin_LED3| Pin_LED4;
    GPIO_Init(GPIO_LED, &GPIO_InitStructure);

    GPIO_SetBits(GPIO_LED, Pin_LED1);
    GPIO_SetBits(GPIO_LED, Pin_LED2);
    GPIO_SetBits(GPIO_LED, Pin_LED3);
    GPIO_SetBits(GPIO_LED, Pin_LED4);
}

/*----------------------------------------------------------
 + 实现功能：控制LED发光亮度 由任务调度调用周期1ms
 + 调用参数功能：发光亮度数组 0-20
----------------------------------------------------------*/
void Call_LED_show( u8 duty[4] )
{
    /* 计时值 */
    static u8 LED_cnt[4];

    /* 依次控制4个LED */
    for(u8 i=0; i<4; i++)
    {
        /* 计时器用于比较 */
        if( LED_cnt[i] < 19 )
            LED_cnt[i]++;
        else
            LED_cnt[i] = 0;

        /* LED开启状态控制 */
        if( LED_cnt[i] < duty[i] )
        {
            /* 依次控制4个LED */
            switch(i)
            {
            case 0:
                LED1_ON;
                break;
            case 1:
                LED2_ON;
                break;
            case 2:
                LED3_ON;
                break;
            case 3:
                LED4_ON;
                break;
            }
        }
        /* LED关断状态控制 */
        else
        {
            /* 依次控制4个LED */
            switch(i)
            {
            case 0:
                LED1_OFF;
                break;
            case 1:
                LED2_OFF;
                break;
            case 2:
                LED3_OFF;
                break;
            case 3:
                LED4_OFF;
                break;
            }
        }
    }
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
