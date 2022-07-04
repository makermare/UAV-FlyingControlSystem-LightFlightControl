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
		loop.check_flag = 1;//该标志位在循环的最后被清零
	}
	LED_1ms_DRV( );	//20级led渐变显示
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
	RC_Duty(inner_loop_time , Rc_Pwm_In );				// 遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
	test[1] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms()
{
	float outer_loop_time;
	
	outer_loop_time = Get_Cycle_T(2)/1000000.0f;								//获取外环准确的执行周期
	
	test[2] = GetSysTime_us()/1000000.0f;

 	CTRL_2(outer_loop_time); 											// 外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	
	test[3] = GetSysTime_us()/1000000.0f;
}

void Duty_10ms()
{	
   LIGHT_AK8975_Read();			//获取电子罗盘数据	
}
//----------------------------------------全局变量----------------------------------------//
extern int16_t  my_data[10];      //分别存储 偏转角度(度)  X轴位移CM  Y轴位移CM
extern uint8_t RXOVER;            //接收数据完成标志
u8     PixFlow_TrackLine_flag=0;  //通过遥控器6通道进行改变模式

float PixFlow_Roll=0;             //定点模式下给出期望角度
float PixFlow_Pitch=0;

float X_Offset,Y_Offset;
float X_Speed,Y_Speed;

void Duty_20ms()
{
	Parameter_Save();
}

//开始进行寻线控制
u8 start_flag=0;//开始寻线的标志
u8 turn_flag=0;//转弯的标志左转和右转
u8 position_control=1;//位置控制就是进行定点标志=1  进行=0不进行
uint32_t runtime1=0;
int16_t  yaw_offset=0;
float move_front,move_back,move_right,turn_right,turn_left;//运动指令
u8 turn_time=0;
void Track_Line(void);

void Duty_50ms()
{	
	mode_check(CH_filter,mode_value);
	LED_Duty();								//LED任务
	Ultra_Duty();
  Track_Line();
}
//自主寻线函数

void Track_Line(void)
{
	if(mode_state==2)//mode_state==2在定高模式下，进入自主飞行
	{  
		 if(runtime1>1500)//防止数据越界
		 {
				  runtime1=0;
		 }
		 if((ultra.height>30)&&(start_flag==0))//定高悬停5s
		 {
				runtime1++;
				if(runtime1==100)//5s
				{
					 runtime1=0;
					 start_flag=1;//定高完成，开始进入自主飞行
					 turn_left=Yaw;
				}
		 }
		 else if(start_flag>=1&&start_flag!=4)
		 {
				runtime1++;
				if((start_flag==1)&&(runtime1==2))//开始以0.5度倾角向前
				{
					 turn_flag=0;
					 move_front=-0.2f;//-1度
					 position_control=2;//不进行定点控制
					 runtime1=0;
					 start_flag=2;
				}
				else if((start_flag==2)&&(turn_flag==1))//检测到直角就停止前进，刹车1.5度 持续时间1.5s
				{
				   // move_front=1.2f;//检测到直角就停止前进
					  start_flag=3;
					  position_control=0;//开始进行定点控制
					  runtime1=0;
				}
				else if((start_flag==3)&&(runtime1>=1)&&(runtime1<=24))//转弯之前定点4.5s,要么刹车之后再定点，要么定点和刹车一起进行
				{
					  move_right=-1.5f;
					  turn_left=turn_left-(s16)(150*0.5f )*0.05;
					  if(turn_left<-180)
					  {
						  turn_left=360-180;
					  }
				}
				else if((start_flag==3)&&(runtime1==25))
				{
					   start_flag=1;
					   move_front=-0.2f;//-1度
					   position_control=2;
					   move_right=0;
					   runtime1=0;
				}
			}
	}
	else//非寻线模式
	{ 
		start_flag=0;
		move_front=0;
		move_right=0;
		turn_left=0;
		turn_flag=0;
  }
}



























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




	
	

