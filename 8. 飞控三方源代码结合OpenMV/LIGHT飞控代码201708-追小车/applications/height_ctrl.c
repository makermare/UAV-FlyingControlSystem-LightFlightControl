#include "height_ctrl.h"
#include "baro_ctrl.h"
#include "mymath.h"
#include "filter.h"
#include "rc.h"
#include "PID.h"
#include "ctrl.h"
#include "include.h"
#include "fly_mode.h"

float	set_height_e,set_height_em,
			set_speed_t,set_speed,exp_speed,fb_speed,
			exp_acc,fb_acc,fb_speed,fb_speed_old;
//--------------------------------------光流定点变量--------------------------------------------------------//
float  exp_speed_x,exp_speed_y;
extern int16_t  my_data[10];        //???? ????(?)  X???CM  Y???CM
extern uint8_t RXOVER;             //????????
extern u8     PixFlow_TrackLine_flag;  //?????6????????

extern float PixFlow_Roll;   //???????????
extern float PixFlow_Pitch;

extern float X_Offset,Y_Offset;
extern float X_Speed,Y_Speed;

_hc_value_st hc_value;

u8 thr_take_off_f = 0;
u8 auto_take_off,auto_land;
float height_ref;

float auto_take_off_land(float dT,u8 ready)
{
	static u8 back_home_old;
	static float thr_auto;
	
	if(ready==0)
	{
		height_ref = hc_value.fusion_height;
		auto_take_off = 0;
	}
	
	if(Thr_Low == 1 && fly_ready == 0)
	{
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //起飞之前，并且解锁之前，非返航模式拨到返航模式
		{
				if(auto_take_off==0)  //第一步，自动起飞标记0->1
				{
					auto_take_off = 1;
				}
		}
	}
	
	switch(auto_take_off)
	{
		case 1:
		{
			if(thr_take_off_f ==1)
			{
				auto_take_off = 2;
			}
			break;
		}
		case 2:
		{
			if(hc_value.fusion_height - height_ref>500)
			{
				if(auto_take_off==2) //已经触发自动起飞
				{
					auto_take_off = 3;
				}
			}
		}
		default:break;
	}

	if(auto_take_off == 2)
	{
		thr_auto = 200;
	}
	else if(auto_take_off == 3)
	{
		thr_auto -= 200 *dT;
	
	}
	
	thr_auto = LIMIT(thr_auto,0,300);
	
	back_home_old = mode_value[BACK_HOME]; //记录模式历史
		
	return (thr_auto);
}
	

//高度部分PID
_PID_arg_st h_acc_arg;
_PID_arg_st h_speed_arg;
_PID_arg_st h_height_arg;

_PID_arg_st x_speed_arg;
_PID_arg_st y_speed_arg;

_PID_arg_st x_offset_arg;
_PID_arg_st y_offset_arg;

_PID_val_st h_acc_val;
_PID_val_st h_speed_val;
_PID_val_st h_height_val;

//----------------------------------------------光流定点部分PID--------------------------------------------------//
_PID_val_st x_speed_val;
_PID_val_st x_offset_val;

_PID_val_st y_speed_val;
_PID_val_st y_offset_val;
//----------------------------------------------光流定点部分PID--------------------------------------------------//


void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//比例系数
	h_acc_arg.ki = 0.02f *pid_setup.groups.hc_sp.kp;				//积分系数
	h_acc_arg.kd = 0;				//微分系数
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;
  //定高速度环
	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//比例系数
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//积分系数
	h_speed_arg.kd = 0.0f;				//微分系数
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;
  
	//定高高度环
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//比例系数
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//积分系数
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//微分系数
	h_height_arg.k_pre_d=0.01f;
	h_height_arg.inc_hz =20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;	
	//摄像头速度环PID控制
  x_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//比例系数
	x_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//积分系数
	x_speed_arg.kd = 0.0f;				//微分系数
	x_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	x_speed_arg.inc_hz = 20;
	x_speed_arg.k_inc_d_norm = 0.8f;
	x_speed_arg.k_ff = 0.5f;
	
  y_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//比例系数
	y_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//积分系数
	y_speed_arg.kd = 0.0f;				//微分系数
	y_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	y_speed_arg.inc_hz = 20;
	y_speed_arg.k_inc_d_norm = 0.8f;
	y_speed_arg.k_ff = 0.5f;		

  //摄像头位置环PID控制
	y_offset_arg.kp = 2.25f *pid_setup.groups.hc_height.kp;				//比例系数
	y_offset_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//积分系数
	y_offset_arg.kd = 2.5f *pid_setup.groups.hc_height.kd;				//微分系数
	y_offset_arg.k_pre_d=0.01f;
	y_offset_arg.inc_hz =20;
	y_offset_arg.k_inc_d_norm = 0.5f;
	y_offset_arg.k_ff =0;
	
	//摄像头位置环PID控制
	x_offset_arg.kp = 2.25f*pid_setup.groups.hc_height.kp;				//比例系数
	x_offset_arg.ki = 0.0f*pid_setup.groups.hc_height.ki;				//积分系数
	x_offset_arg.kd = 2.5f*pid_setup.groups.hc_height.kd;				//微分系数
	x_offset_arg.k_pre_d=0.01f;
	x_offset_arg.inc_hz =20;
	x_offset_arg.k_inc_d_norm=0.5f;
	x_offset_arg.k_ff =0;
}

float thr_set,thr_pid_out,thr_out,thr_take_off,tilted_fix;

float en_old;
u8 ex_i_en_f,ex_i_en;
extern u8 position_control;
extern u8   PixFlow_TrackLine_flag; 
extern float angle_max;
extern float speed_max;
extern u8 start_flag;
float Height_Ctrl(float T,float thr,u8 ready,float en)
{
	static u8 step,speed_cnt,height_cnt;
	if(ready==0)
	{
		ex_i_en = ex_i_en_f = 0;
		en = 0;
		thr_take_off = 0;
		thr_take_off_f = 0;
	}
	switch(step)
	{
		case 0:
		{
			break;
		}
		case 1:
		{
			step = 2;
			break;
		}
		case 2:
		{
			step = 3;
			break;
		}
		case 3:
		{
			step = 4;
			break;
		}	
		case 4:
		{
			step = 0;
			break;
		}	
		default:break;	
	}
	
	/*飞行中初次进入定高模式切换处理*/
	if(ABS(en - en_old) > 0.5f)//从非定高切换到定高
	{
		if(thr_take_off<10)//未计算起飞油门
		{
			if(thr_set>-150)
			{
				thr_take_off=390;//原来的数值是400
			}
		}
		en_old = en;
	}
	 if(mode_state==2)//气压计模式标志，代表一键起飞定高
	 {
			if(PixFlow_TrackLine_flag==0)//模式1定高悬停降落
			{
				  if(ultra.height<=70)
		    	{
				       thr=650;//上升
			    }
					else if(ultra.height>70&&ultra.height<=95)
					{
						 thr=650-(ultra.height-70)*4.0f;
					}
					else if(ultra.height>95&&ultra.height<105)
					{
						 thr=551;
					}
					else if(ultra.height>=105&&ultra.height<115)
					{
						 thr=500;
					}
					else if(ultra.height>=115&&ultra.height<=125)
					{
						 thr=450;
					}
					else if(ultra.height>=125)
					{
						 thr=420;
					} 
		 }
		
    if(PixFlow_TrackLine_flag==1||PixFlow_TrackLine_flag==2)//模式1定高悬停降落
		{
				  if(ultra.height<=60)
		    	{
				       thr=650;//上升
			    }
					else if(ultra.height>60&&ultra.height<=85)
					{
						 thr=650-(ultra.height-60)*4.0f;
					}
					else if(ultra.height>85&&ultra.height<95)
					{
						 thr=551;
					}
					else if(ultra.height>=95&&ultra.height<105)
					{
						 thr=500;
					}
					else if(ultra.height>=105&&ultra.height<=115)
					{
						 thr=450;
					}
					else if(ultra.height>=115)
					{
						 thr=420;
					} 
		  }
	}	
		if(mode_state==1||start_flag==5)//超声波模式标志，代表一键降落
		{
		    thr=300;//下降
		}
	//油门设定阈值-+500，，这个值用来设定是上升还是下降>0  上升，，，<0 下降
	thr_set=my_deathzoom_2(my_deathzoom((thr-500),0,40),0,10);
	
	if(thr_set>0)//对应油门遥杆大于1500
	{
		//设置油门上升速度
		set_speed_t = thr_set/450*MAX_VERTICAL_SPEED_UP;//这里的速度为正值
		if(thr_set>100)//对应油门遥杆1640
		{
			ex_i_en_f = 1;//这个标志具体是做什么的呢？
			if(!thr_take_off_f)//起飞标志为0
			{
				thr_take_off_f = 1; //用户可能想要起飞
				thr_take_off = 390; //直接赋值一次
			}
		}
	}
	else//油门遥杆值小于1460，想要下降
	{
		if(ex_i_en_f == 1)//期望积分是使能标志
		{
			ex_i_en = 1;//期望积分使能
		}
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;//这里的速度为  负   值
	}
	//限制高度爬升率
	set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
	//低通滤波
	LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);
	//对低通滤波之后的爬升速度进行限制幅度
	set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
  /////////////////////////////////////////////////////////////////////////////////	
	baro_ctrl(T,&hc_value); //高度数据获取： 气压计数据
  /////////////////////////////////////////////////////////////////////////////////		
	
	//当达到期望高度的时候  设定速度等于0
	
	//计算高度误差（可加滤波)
	// MS气压计数据分析出来的垂直速率和期望的攀爬速率进行做差*T=设定期望高度
	set_height_em += (set_speed - hc_value.m_speed) *T;
	set_height_em  = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);
	
	//超声波和气压计融合的数据分析出来的垂直速率和期望的攀爬速率进行做差*T=设定期望高度
	set_height_e += (set_speed - 1.05f *hc_value.fusion_speed) *T;
	set_height_e  = LIMIT(set_height_e,-5000 *ex_i_en,5000 *ex_i_en);
		//融合两个气压计和超声波的数据set_height_e=out
	LPF_1_(0.05f,T,set_height_em,set_height_e);
  //修改区域
	//set_height_e=0;//mm为单位，设置期望高度
	//修改区域
	
	if(en<0.1f)
	{
		exp_speed=hc_value.fusion_speed;//期望速度=传感器测量的速度
		exp_acc  =hc_value.fusion_acc;  //期望加速度=传感器融合的加速度
	}
  /////////////////////////////////////////////////////////////////////////////////	
	//加速度积分限制幅度
	float acc_i_lim;
	acc_i_lim =safe_div(150,h_acc_arg.ki,0);
	fb_speed_old = fb_speed;
	fb_speed = hc_value.fusion_speed;
	fb_acc = safe_div(fb_speed - fb_speed_old,T,0);
	
	//(1)   加速度环控制，输入值为期望加速度 控制周期=2ms
	thr_pid_out = PID_calculate(T,       //周期
														exp_acc,	  //前馈
														exp_acc,		//期望值（设定值）
														fb_acc,			//反馈值
														&h_acc_arg, //PID参数结构体
														&h_acc_val,	//PID数据结构体
														acc_i_lim*en			//integration limit，积分限幅
														);			//输出		
	//起飞油门
	if(h_acc_val.err_i > (acc_i_lim * 0.2f))
	{
		if(thr_take_off<THR_TAKE_OFF_LIMIT)
		{
			thr_take_off+=150*T;
			h_acc_val.err_i -= safe_div(150,h_acc_arg.ki,0) *T;
		}
	}
	else if(h_acc_val.err_i < (-acc_i_lim * 0.2f))
	{
		if(thr_take_off>0)
		{
			thr_take_off -= 150 *T;
			h_acc_val.err_i+= safe_div(150,h_acc_arg.ki,0) *T;
		}
	}
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //一半
	
	//油门补偿
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45度内补偿
	
	//油门输出
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );
	
	thr_out = LIMIT(thr_out,0,1000);

  ///////////////////////////////////////////////////////////////////////////
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{
    //(2)速度环控制控制周期=20ms，注意输入的值为 ：  期望速度+设定速度    ，设定速度也是通过油门值来控制的
		exp_acc = PID_calculate(dT,           //周期
														exp_speed,		//前馈
														(set_speed + exp_speed),				//期望值（设定值）
														hc_value.fusion_speed,			    //反馈值
														&h_speed_arg, //PID参数结构体
														&h_speed_val,	//PID数据结构体
														500*en			  //integration limit，积分限幅
														 );           //输出	
		exp_acc=LIMIT(exp_acc,-3000,3000); 
		
		//------------------------------------光流定点控制速度环20ms---------------------------------------------------//
		 if((ultra.height>25)&&(position_control==1))
		 {
			  exp_speed_x=PID_calculate(dT,           //周期
																0,				      //前馈
																0,				      //期望值(设定值)
																X_Offset,			  //反馈值
																&x_offset_arg,  //PID参数结构体
																&x_offset_val,	//PID数据结构体
																1500 *en			  //integration limit，积分限幅
																 );             //输出	
			
			  exp_speed_y=PID_calculate(dT,           //周期
																0,				      //前馈
																0,				      //期望值（设定值）
																Y_Offset,			  //反馈值
																&y_offset_arg,  //PID参数结构体
																&y_offset_val,	//PID数据结构体
																1500*en			//integration limit，积分限幅
																 );			//输出	
			 PixFlow_Roll  = LIMIT(exp_speed_x/300.0f,-speed_max,speed_max);
			 PixFlow_Pitch =-LIMIT(exp_speed_y/300.0f,-speed_max,speed_max);
		 }
		 else 
		 {
			  PixFlow_Roll=0;
			  PixFlow_Pitch=0;
		 }
		//------------------------------------光流定点控制速度环20ms---------------------------------------------------//
		dT2 += dT;
		height_cnt++;
		//(3) 高度环控制输入值为设定目标高度  设定目标高度是通过油门值改变设定速度 ，设定速度和实际速度对时间积分得到设定高度
		if(height_cnt>=10)//200ms 
		{
				 exp_speed = PID_calculate(dT2,       //周期
																		0,				//前馈
																		0,				//期望值（设定值）
																		-set_height_e, //反馈值
																		&h_height_arg, //PID参数结构体
																		&h_height_val, //PID数据结构体
																		1500*en			 //integration limit，积分限幅
																		 );			       //输出	
				 
				 exp_speed=LIMIT(exp_speed,-300,300);	
				 dT2 = 0;
				 height_cnt = 0;
		 }
		 speed_cnt = 0;
		 dT=0;				
	}
  /////////////////////////////////////////////////////////////////////////////////	
	if(step==0)
	{
		step = 1;
	}
	if(en < 0.1f)
	{
		return (thr);
	}
	else
	{
		return (thr_out);
	}
}


