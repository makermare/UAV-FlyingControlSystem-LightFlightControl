#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "ms5611.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "fly_mode.h"

s16 loop_cnt;


loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	
		LED_1ms_DRV( );								//20级led渐变显示
}

void Duty_1ms()
{
	Get_Cycle_T(1)/1000000.0f;

	LIGHT_DT_Data_Exchange();												//数传通信定时调用
}

float test[5];
void Duty_2ms()
{
	float inner_loop_time;
	
	inner_loop_time = Get_Cycle_T(0)/1000000.0f; 						//获取内环准确的执行周期
	
	test[0] = GetSysTime_us()/1000000.0f;
	
	MPU6050_Read(); 															//读取mpu6轴传感器

	MPU6050_Data_Prepare(inner_loop_time );			//mpu6轴传感器数据处理
	
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *inner_loop_time,mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,&Roll,&Pitch,&Yaw);

	CTRL_1( inner_loop_time ); 										//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2)/1000000.0f;								//获取外环准确的执行周期
	
	test[2] = GetSysTime_us()/1000000.0f;

 	CTRL_2( outer_loop_time ); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	
	test[3] = GetSysTime_us()/1000000.0f;
}

void Duty_10ms()
{
	  LIGHT_AK8975_Read();			//获取电子罗盘数据	
}


//我们把自主寻线的程序添加到20ms的控制周期内
//uint32_t sysTickUptime
uint32_t runtime1;

u8 start_flag=0;//开始寻线的标志

float move_front,move_back,turn_right,turn_left;//运动指令

void Track_Line(void);

void Duty_20ms()
{
	Parameter_Save();
	Track_Line();
}
//自主寻线函数
void Track_Line(void)
{
	if(mode_state==2)//mode_state==2在定高模式下，进入自主飞行
	{
		 if((ultra.height>40)&&(ultra.height<55)&&(start_flag==0))
		 {
				runtime1++;
				if(runtime1==100)//2s
				{
					 runtime1=0;
					 start_flag=1;//定高50完成，开始进入自主飞行
				}
		 }
		 else if(start_flag>=1&&start_flag!=7)
		 {
				runtime1++;
				if((start_flag==1)&&(runtime1==10))//刚开始进入自主飞行标志,if只进入一次
				{
					 move_front=-4;//5度
					 turn_left=Yaw;
					 start_flag=2;
					 runtime1=0;
				}
				
				else if((start_flag==2)&&(runtime1==120))//
				{			
					  move_front=3;//停止前进，刹车
					  //turn_left=Yaw;//期望偏航角不改变
				    //start_flag=3;
					  //runtime1=0;
				}
				else if((start_flag==2)&&(runtime1==200))//1.6s
				{
				    move_front=0;
				}
				else if((start_flag==2)&&(runtime1==230))
				{
				   start_flag=3;
					 runtime1=0;
				}
				else if((start_flag==3)&&(runtime1<=65))//九十度转弯
				{
						turn_left +=(s16)(150*0.5f )*0.02;
					  if(turn_left>180)
						{
						  turn_left=180-360;
						}
				}
				
				else if((start_flag==3)&&(runtime1==150))//3s
				{
					 //turn_left=Yaw;
					 move_front=-4;//2度
					 start_flag=4;
					 runtime1=0;
				}
				
				else if((start_flag==4)&&(runtime1==100))//前进2s
				{			
					 move_front=3;
					// start_flag=5;
					// runtime1=0;
				}
			  else if((start_flag==4)&&(runtime1==180))
				{
				   move_front=0;
				}
				else if((start_flag==4)&&(runtime1==230))
				{
				    start_flag=5;
					  runtime1=0;
				}
				else if((start_flag==5)&&(runtime1<=65))
				{
						turn_left+=(s16)(150*0.5f)*0.02;
					  if(turn_left>180)
						{
						  turn_left=180-360;
						}
				}
				else if((start_flag==5)&&(runtime1==150))
				{
						move_front=-4;
						//turn_left=Yaw;
						runtime1=0;
						start_flag=6;
				}
				else if((start_flag==6)&&(runtime1==100))
				{
					 move_front=3;
				}
			  else if((start_flag==6)&&(runtime1==180))
				{
				  move_front=0;
				}
				else if((start_flag==6)&&(runtime1==250))
				{
				   runtime1=0;
					 start_flag=7;
				}
			}
		}
	else//非寻线模式
	{ 
		start_flag=0;
		move_front=0;
		turn_left=0;
  }
}
void Duty_50ms()
{
	//Mode();	
	mode_check(CH_filter,mode_value);
	LED_Duty();								//LED任务
	Ultra_Duty();
}

//仍然是以 AUX1 5通道进行自主寻线控制模式的切换

void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{
	if(loop.check_flag ==1)
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if(loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if(loop.cnt_5ms>=5)
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if(loop.cnt_10ms >=10)
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >=20)
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >=50)
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		loop.check_flag = 0;		//循环运行完毕标志
	}
}


	
	

