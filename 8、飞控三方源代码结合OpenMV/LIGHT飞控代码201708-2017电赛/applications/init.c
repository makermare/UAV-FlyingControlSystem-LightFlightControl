#include "include.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"
#include "ctrl.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"

u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 	//滴答时钟
	
	I2c_Soft_Init();					//初始化模拟I2C
	
	PWM_IN_Init();						//初始化接收机采集功能
	
	PWM_Out_Init(400);				//初始化电调输出功能	
	
	Usb_Hid_Init();						//飞控usb接口的hid初始化
	
	MS5611_Init();						//气压计初始化
	
	Delay_ms(400);						//延时
	
	MPU6050_Init(20);   			//加速度计、陀螺仪初始化，配置20hz低通
	
	LED_Init();								//LED功能初始化
	
	Usart2_Init(38400);			//串口2初始化，函数参数为波特率

  Usart1_Init(115200);      //串口1光流初始化
	
	Para_Init();							//参数初始化
	
	Delay_ms(100);						//延时
	
	Ultrasonic_Init();   			//超声波初始化
	
	ak8975_ok = !(LIGHT_AK8975_Run());
	
	if( !mpu6050_ok )
	{
		LED_MPU_Err();
	}
	else if( !ak8975_ok )
	{
		LED_Mag_Err();
	}
	else if( !ms5611_ok )
	{
		LED_MS5611_Err();
	}
	
	aircraft_mode_led(MAXMOTORS);
	
	Cycle_Time_Init();
	
 	return (1);
}

