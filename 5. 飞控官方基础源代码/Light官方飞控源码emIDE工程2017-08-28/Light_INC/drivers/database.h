/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved.
 + 文件名  ：database.h
 + 描述    ：数据结构头文件
代码尽量做到逐行注释 代码有问题，及时加群交流 作者有偿支持对开源代码的完善 */
#ifndef _DATABASE_H_
#define _DATABASE_H_

#include "stm32f4xx.h"

/* 加速度计、陀螺仪数据枚举体 */
enum
{
    A_X = 0,
    A_Y ,
    A_Z ,
    G_Y ,
    G_X ,
    G_Z ,
    TEM ,
    ITEMS ,
};

/* 遥控控制通道数组枚举体 */
enum
{
    ROL= 0,
    PIT ,
    THR ,
    YAW ,
    AUX1 ,
    AUX2 ,
    AUX3 ,
    AUX4 ,
};

/* PID参数结构体 */
typedef struct
{
    float kp;
    float kd;
    float ki;
    float kdamp;
} pid_t;

/* XYZ结构体 */
typedef struct
{
    float x;
    float y;
    float z;
} xyz_f_t;

/* xyz结构体 */
typedef struct
{
    s16 x;
    s16 y;
    s16 z;

} xyz_s16_t;

/* 传感器数据共用体 */
typedef union
{
    uint8_t raw_data[64];
    struct
    {
        xyz_f_t Accel;
        xyz_f_t Gyro;
        xyz_f_t Mag;
        xyz_f_t vec_3d_cali;
        uint32_t Baro;
        float Acc_Temperature;
        float Gyro_Temperature;
    } Offset;
} sensor_setup_t;

/* PID在3轴上的控制量结构体 */
typedef  struct
{
    pid_t roll;
    pid_t pitch;
    pid_t yaw;
} pid_group_t;

/* PID总的控制量共用体 */
typedef union
{
    uint8_t raw_data[192];
    struct
    {
        pid_group_t ctrl1;
        pid_group_t ctrl2;
        pid_t hc_sp;
        pid_t hc_height;
        pid_t ctrl3;
        pid_t ctrl4;
    } groups;

} pid_setup_t;

/* 磁力计数组：采样值,偏移值,纠正后的值 */
typedef struct
{
    xyz_s16_t   Mag_Adc;		//采样值
    xyz_f_t     Mag_Offset;		//偏移值
    xyz_f_t 	Mag_Val;	//纠正后的值
} ak8975_t;

/* IMU传感器mpu6050数组*/
typedef struct
{
    /* 传感器校准标志 */
    char Acc_CALIBRATE;
    char Gyro_CALIBRATE;
    /* IMU数据 */
    xyz_s16_t Acc_I16;
    xyz_s16_t Gyro_I16;
    xyz_f_t Acc;
    xyz_f_t Gyro;
    xyz_f_t Gyro_deg;
    /* IMU补偿 */
    xyz_f_t Acc_Offset;
    xyz_f_t Gyro_Offset;
    xyz_f_t vec_3d_cali;
    /* 温度补偿 */
    float Acc_Temprea_Offset;
    float Gyro_Temprea_Offset;
    float Gyro_Temprea_Adjust;
    float ACC_Temprea_Adjust;
    /* 读取温度信息及滤波 */
    s16 Tempreature;
    float TEM_LPF;
    float Ftempreature;
} MPU6050_STRUCT;

#endif
/* ©2015-2016 Beijing Bechade Opto-Electronic, Co.,Ltd. All rights reserved. */
