/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：driver_led.c
 + 描述    ：led指示灯状态控制
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "stm32f4xx.h"
#include "driver_led.h"
#include "time.h"
#include "rc.h"
#include "driver_ak8975.h"
#include "driver_GPS.h"

/* 引用定高模式作为状态指示 */
extern u8 height_ctrl_mode;
/* LED状态指示 */
u8 LED_status;
/* 每个LED亮度范围是0-20 */
u8 LED_Brightness[4];

/*----------------------------------------------------------
 + 实现功能：由任务调度调用周期50ms
----------------------------------------------------------*/
void Call_LED_duty()
{
    /* 计时变量 */
    static s16 led_temp=0;
    /* 亮暗的切换 */
    static u8 f;

    /* 判断指示状态 */
    if(!Mag_CALIBRATED)
        LED_status = height_ctrl_mode;
    /* 磁力计校准特殊指示 */
    else LED_status = 0x0A;

    /* 自稳模式状态指示 */
    if(LED_status==0)
    {
        /* 未解锁状态指示 */
        if(!unlocked_to_fly)
        {
            /* 由暗变亮 */
            if( f )
            {
                led_temp += 4;
                if(led_temp>100) f = 0;
            }
            /* 由亮变暗 */
            else
            {
                led_temp -= 4;
                if(led_temp < 0 ) f = 1;
            }
            /* 指示灯状态数组赋值 */
            LED_Brightness[1] = led_temp/10;
            LED_Brightness[2] = led_temp/10;
            LED_Brightness[3] = led_temp/10;
        }
        /* 解锁后状态指示 */
        else
        {
            /* 指示灯状态数组赋值 */
            LED_Brightness[1] = 12;
            LED_Brightness[2] = 12;
            LED_Brightness[3] = 12;
        }
    }
    /* 气压定高模式状态指示 */
    else if(LED_status==1)
    {
        /* 未解锁状态指示 */
        if(!unlocked_to_fly)
        {
            /* 由暗变亮 */
            if( f )
            {
                led_temp += 4;
                if(led_temp>100) f = 0;
            }
            /* 由亮变暗 */
            else
            {
                led_temp -= 4;
                if(led_temp < 0 ) f = 1;
            }
            /* 指示灯状态数组赋值 */
            LED_Brightness[1] = 0;
            LED_Brightness[2] = led_temp/10;
            LED_Brightness[3] = led_temp/10;
        }
        /* 解锁后状态指示 */
        else
        {
            /* 指示灯状态数组赋值 */
            LED_Brightness[1] = 0;
            LED_Brightness[2] = 12;
            LED_Brightness[3] = 12;
        }
    }
    /* 超声波定高模式状态指示 */
    else if(LED_status==2)
    {
        /* 未解锁状态指示 */
        if(!unlocked_to_fly)
        {
            /* 由暗变亮 */
            if( f )
            {
                led_temp += 4;
                if(led_temp>100) f = 0;
            }
            /* 由亮变暗 */
            else
            {
                led_temp -= 4;
                if(led_temp < 0 ) f = 1;
            }
            /* 指示灯状态数组赋值 */
            LED_Brightness[1] = led_temp/10;
            LED_Brightness[2] = led_temp/10;
            LED_Brightness[3] = 0;
        }
        /* 解锁后状态指示 */
        else
        {
            /* 指示灯状态数组赋值 */
            LED_Brightness[1] = 12;
            LED_Brightness[2] = 12;
            LED_Brightness[3] = 0;
        }
    }
    /* 磁力计校准模式状态指示 */
    else if(LED_status==0x0A)
    {
        /* 由暗变亮 */
        if( f )
        {
            led_temp += 12;
            if(led_temp>100) f = 0;
        }
        /* 由亮变暗 */
        else
        {
            led_temp -= 12;
            if(led_temp < 0 ) f = 1;
        }
        /* 指示灯状态数组赋值 */
        LED_Brightness[1] = led_temp/10;
        LED_Brightness[2] = 0;
        LED_Brightness[3] = led_temp/10;
    }
    if(gpsx.fixmode>=2)
        LED_Brightness[0] = 12;
    else
        LED_Brightness[0] = 0;
}

/*----------------------------------------------------------
 + 实现功能：MPU故障指示
----------------------------------------------------------*/
void LED_MPU_Err(void)
{
    LED1_ON;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    while(1)
    {
        LED2_ON;
        Delay_ms(150);
        LED2_OFF;
        Delay_ms(150);
    }
}

/*----------------------------------------------------------
 + 实现功能：磁力计故障指示
----------------------------------------------------------*/
void LED_Mag_Err(void)
{
    LED1_ON;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    while(1)
    {
        LED3_ON;
        Delay_ms(150);
        LED3_OFF;
        Delay_ms(150);
    }
}

/*----------------------------------------------------------
 + 实现功能：气压计故障指示
----------------------------------------------------------*/
void LED_MS5611_Err(void)
{
    LED1_ON;
    LED2_OFF;
    LED3_OFF;
    LED4_OFF;
    while(1)
    {
        LED4_ON;
        Delay_ms(150);
        LED4_OFF;
        Delay_ms(150);
    }
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
