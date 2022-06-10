/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：scheduler.c
 + 描述    ：任务调度
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#include "stm32f4xx.h"
#include "scheduler.h"
#include "time.h"
#include "driver_ak8975.h"
#include "driver_led.h"
#include "rc.h"
#include "imu.h"
#include "device_pwm_in.h"
#include "ctrl.h"
#include "driver_ms5611.h"
#include "driver_parameter.h"
#include "driver_ultrasonic.h"
#include "driver_GPS.h"
#include "height_ctrl.h"
#include "data_transfer.h"
#include "position_control.h"

/* 循环计数结构体 */
loop_t loop;

/*----------------------------------------------------------
 + 实现功能：1ms周期任务
----------------------------------------------------------*/
void Duty_1ms()
{
    /* 调用渐变显示 */
    Call_LED_show( LED_Brightness );
    /* 调用数传通信 */
    Call_Data_transfer();
}

/*----------------------------------------------------------
 + 实现功能：2ms周期任务
----------------------------------------------------------*/
void Duty_2ms()
{
    /* 调用0号计时通道，用于计算两侧调用的时间间隔 */
    float inner_loop_time = Call_timer_cycle(0);
    /* mpu6轴传感器数据处理 */
    Call_MPU6050_Data_Prepare( inner_loop_time );
    /* 内环角速度控制 */
    CTRL_angular_velocity( inner_loop_time );
    /* 遥控器通道数据处理 */
    Call_RadioContrl( inner_loop_time );
}

/*----------------------------------------------------------
 + 实现功能：5ms周期任务
----------------------------------------------------------*/
void Duty_5ms()
{
    /* 调用1号计时通道，用于计算两侧调用的时间间隔 */
    float outer_loop_time = Call_timer_cycle(1);
    /* IMU更新姿态:ROL,PIT,YAW姿态角 */
    Call_IMUupdate(0.5f *outer_loop_time);
    /* 外环角度控制 */
    CTRL_attitude ( outer_loop_time );
}

/*----------------------------------------------------------
 + 实现功能：10ms周期任务
----------------------------------------------------------*/
void Duty_10ms()
{
    /* 气压计数据处理 */
    if( MS5611_Update() ) baro_ctrl_start = 1;
    /* 磁力计（电子罗盘）数据处理 */
    Call_AK8975();
}

/*----------------------------------------------------------
 + 实现功能：20ms周期任务
----------------------------------------------------------*/
void Duty_20ms()
{
    /* 保存参数 */
    Parameter_Save();
}

/*----------------------------------------------------------
 + 实现功能：50ms周期任务
----------------------------------------------------------*/
void Duty_50ms()
{
    /* 飞行模式检测及切换 */
    Call_RadioControl_Mode();
    /* 控制LED任务 */
    Call_LED_duty();
}

/*----------------------------------------------------------
 + 实现功能：100ms周期任务
----------------------------------------------------------*/
void Duty_100ms()
{
    /* 调用2号计时通道，用于计算两侧调用的时间间隔 */
    float position_loop_time=Call_timer_cycle(2);
    /* 超声波模块 */
    Call_Ultrasonic();
    /* GPS模块 */
    Call_GPS();
    /* 位置控制 */
    PositionControl_Mode(position_loop_time);
}

/*----------------------------------------------------------
 + 实现功能：主循环 由主函数调用
----------------------------------------------------------*/
void Main_Loop()
{
    /* 循环周期为1ms */
    if( loop.check_flag == 1 )
    {
        Duty_1ms();     //周期1ms的任务
        /* 判断每个不同周期的执行任务执行条件 */
        if( loop.cnt_2ms >= 2 )
        {
            loop.cnt_2ms = 0;
            Duty_2ms();     //周期2ms的任务
        }
        if( loop.cnt_5ms >= 5 )
        {
            loop.cnt_5ms = 0;
            Duty_5ms();     //周期5ms的任务
        }
        if( loop.cnt_10ms >= 10 )
        {
            loop.cnt_10ms = 0;
            Duty_10ms();    //周期10ms的任务
        }
        if( loop.cnt_20ms >= 20 )
        {
            loop.cnt_20ms = 0;
            Duty_20ms();    //周期20ms的任务
        }
        if( loop.cnt_50ms >= 50 )
        {
            loop.cnt_50ms = 0;
            Duty_50ms();    //周期50ms的任务
        }
        if( loop.cnt_100ms >= 100 )
        {
            loop.cnt_100ms = 0;
            Duty_100ms();    //周期50ms的任务
        }
        /* 循环运行完毕 标志清零 */
        loop.check_flag = 0;
    }
}

/*----------------------------------------------------------
 + 实现功能：由Systick定时器调用 周期：1毫秒
----------------------------------------------------------*/
void Call_Loop_timer()
{
    /* 不同周期的执行任务独立计时 */
    loop.cnt_2ms++;
    loop.cnt_5ms++;
    loop.cnt_10ms++;
    loop.cnt_20ms++;
    loop.cnt_50ms++;
    loop.cnt_100ms++;

    /* 如果代码在预定周期内没有运行完 */
    if( loop.check_flag == 1)
        /* 错误次数计数 */
        loop.err_flag ++;
    /* 循环运行开始 标志置1 */
    else loop.check_flag = 1;

    /* 等待数传数据开始发送 */
    static short time_1ms;

    /* 上电延时 标志置1 */
    if(time_1ms == 4096)
    {
        /* 上电延时自动校准气压计 */
        start_height=0;
        /* 上电延时后发送数据 */
        wait_for_translate = 1;
        time_1ms++;
    }
    else if(time_1ms < 4096)
        time_1ms++;
}

/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
