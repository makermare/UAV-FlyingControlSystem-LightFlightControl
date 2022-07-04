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
//--------------------------------------¹âÁ÷¶¨µã±äÁ¿--------------------------------------------------------//
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
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //Æğ·ÉÖ®Ç°£¬²¢ÇÒ½âËøÖ®Ç°£¬·Ç·µº½Ä£Ê½²¦µ½·µº½Ä£Ê½
		{
				if(auto_take_off==0)  //µÚÒ»²½£¬×Ô¶¯Æğ·É±ê¼Ç0->1
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
				if(auto_take_off==2) //ÒÑ¾­´¥·¢×Ô¶¯Æğ·É
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
	
	back_home_old = mode_value[BACK_HOME]; //¼ÇÂ¼Ä£Ê½ÀúÊ·
		
	return (thr_auto);
}
	

//¸ß¶È²¿·ÖPID
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

//----------------------------------------------¹âÁ÷¶¨µã²¿·ÖPID--------------------------------------------------//
_PID_val_st x_speed_val;
_PID_val_st x_offset_val;

_PID_val_st y_speed_val;
_PID_val_st y_offset_val;
//----------------------------------------------¹âÁ÷¶¨µã²¿·ÖPID--------------------------------------------------//


void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//±ÈÀıÏµÊı
	h_acc_arg.ki = 0.02f *pid_setup.groups.hc_sp.kp;				//»ı·ÖÏµÊı
	h_acc_arg.kd = 0;				//Î¢·ÖÏµÊı
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;
  //¶¨¸ßËÙ¶È»·
	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//±ÈÀıÏµÊı
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//»ı·ÖÏµÊı
	h_speed_arg.kd = 0.0f;				//Î¢·ÖÏµÊı
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;
  
	//¶¨¸ß¸ß¶È»·
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//±ÈÀıÏµÊı
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//»ı·ÖÏµÊı
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//Î¢·ÖÏµÊı
	h_height_arg.k_pre_d=0.01f;
	h_height_arg.inc_hz =20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;	
	//¹âÁ÷ËÙ¶È»·PID¿ØÖÆ
  x_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//±ÈÀıÏµÊı
	x_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//»ı·ÖÏµÊı
	x_speed_arg.kd = 0.0f;				//Î¢·ÖÏµÊı
	x_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	x_speed_arg.inc_hz = 20;
	x_speed_arg.k_inc_d_norm = 0.8f;
	x_speed_arg.k_ff = 0.5f;
	
  y_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//±ÈÀıÏµÊı
	y_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//»ı·ÖÏµÊı
	y_speed_arg.kd = 0.0f;				//Î¢·ÖÏµÊı
	y_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	y_speed_arg.inc_hz = 20;
	y_speed_arg.k_inc_d_norm = 0.8f;
	y_speed_arg.k_ff = 0.5f;		

  //¹âÁ÷Î»ÖÃ»·PID¿ØÖÆ
	y_offset_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//±ÈÀıÏµÊı
	y_offset_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//»ı·ÖÏµÊı
	y_offset_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//Î¢·ÖÏµÊı
	y_offset_arg.k_pre_d=0.01f;
	y_offset_arg.inc_hz =20;
	y_offset_arg.k_inc_d_norm = 0.5f;
	y_offset_arg.k_ff =0;
	
	//¹âÁ÷Î»ÖÃ»·PID¿ØÖÆ
	x_offset_arg.kp = 1.5f*pid_setup.groups.hc_height.kp;				//±ÈÀıÏµÊı
	x_offset_arg.ki = 0.0f*pid_setup.groups.hc_height.ki;				//»ı·ÖÏµÊı
	x_offset_arg.kd = 0.05f*pid_setup.groups.hc_height.kd;				//Î¢·ÖÏµÊı
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
	
	/*·ÉĞĞÖĞ³õ´Î½øÈë¶¨¸ßÄ£Ê½ÇĞ»»´¦Àí*/
	if(ABS(en - en_old) > 0.5f)//´Ó·Ç¶¨¸ßÇĞ»»µ½¶¨¸ß
	{
		if(thr_take_off<10)//Î´¼ÆËãÆğ·ÉÓÍÃÅ
		{
			if(thr_set>-150)
			{
				thr_take_off=360;//Ô­À´µÄÊıÖµÊÇ400
			}
		}
		en_old = en;
	}
	 if(mode_state==2)//ÆøÑ¹¼ÆÄ£Ê½±êÖ¾£¬´ú±íÒ»¼üÆğ·É¶¨¸ß
	 {
			//ĞŞ¸Ä²¿·Ö¬ÎÈ¶¨ÔÚ50cmÉÏÏÂ£¬thr=500
			if(ultra.height<=20)
			{
				 thr=650;//ÉÏÉı
			}
		  else if(ultra.height>20&&ultra.height<=25)
		  {
		   thr=650-(ultra.height-20)*20;
      }
		 else if(ultra.height>25&&ultra.height<=40)
		 {
			  thr=551;
		 }
			else if(ultra.height>40&&ultra.height<50)
			{
			   thr=500;
			}
			else if(ultra.height>=50&&ultra.height<=65)
			{
			   thr=450;
			}
			else if(ultra.height>=65)
			{
			   thr=420;
			} 
		}
		if(mode_state==1||start_flag==4)//³¬Éù²¨Ä£Ê½±êÖ¾£¬´ú±íÒ»¼ü½µÂä
		{
		    thr=300;//ÏÂ½µ
		}
	//ÓÍÃÅÉè¶¨ãĞÖµ-+500£¬£¬Õâ¸öÖµÓÃÀ´Éè¶¨ÊÇÉÏÉı»¹ÊÇÏÂ½µ>0  ÉÏÉı£¬£¬£¬<0 ÏÂ½µ
	thr_set=my_deathzoom_2(my_deathzoom((thr-500),0,40),0,10);
	
	if(thr_set>0)//¶ÔÓ¦ÓÍÃÅÒ£¸Ë´óÓÚ1500
	{
		//ÉèÖÃÓÍÃÅÉÏÉıËÙ¶È
		set_speed_t = thr_set/450*MAX_VERTICAL_SPEED_UP;//ÕâÀïµÄËÙ¶ÈÎªÕıÖµ
		if(thr_set>100)//¶ÔÓ¦ÓÍÃÅÒ£¸Ë1640
		{
			ex_i_en_f = 1;//Õâ¸ö±êÖ¾¾ßÌåÊÇ×öÊ²Ã´µÄÄØ£¿
			if(!thr_take_off_f)//Æğ·É±êÖ¾Îª0
			{
				thr_take_off_f = 1; //ÓÃ»§¿ÉÄÜÏëÒªÆğ·É
				thr_take_off = 360; //Ö±½Ó¸³ÖµÒ»´Î
			}
		}
	}
	else//ÓÍÃÅÒ£¸ËÖµĞ¡ÓÚ1460£¬ÏëÒªÏÂ½µ
	{
		if(ex_i_en_f == 1)//ÆÚÍû»ı·ÖÊÇÊ¹ÄÜ±êÖ¾
		{
			ex_i_en = 1;//ÆÚÍû»ı·ÖÊ¹ÄÜ
		}
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;//ÕâÀïµÄËÙ¶ÈÎª  ¸º   Öµ
	}
	//ÏŞÖÆ¸ß¶ÈÅÀÉıÂÊ
	set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
	//µÍÍ¨ÂË²¨
	LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);
	//¶ÔµÍÍ¨ÂË²¨Ö®ºóµÄÅÀÉıËÙ¶È½øĞĞÏŞÖÆ·ù¶È
	set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
  /////////////////////////////////////////////////////////////////////////////////	
	baro_ctrl(T,&hc_value); //¸ß¶ÈÊı¾İ»ñÈ¡£º ÆøÑ¹¼ÆÊı¾İ
  /////////////////////////////////////////////////////////////////////////////////		
	
	//µ±´ïµ½ÆÚÍû¸ß¶ÈµÄÊ±ºò  Éè¶¨ËÙ¶ÈµÈÓÚ0
	
	//¼ÆËã¸ß¶ÈÎó²î£¨¿É¼ÓÂË²¨£©
	// MSÆøÑ¹¼ÆÊı¾İ·ÖÎö³öÀ´µÄ´¹Ö±ËÙÂÊºÍÆÚÍûµÄÅÊÅÀËÙÂÊ½øĞĞ×ö²î*T=Éè¶¨ÆÚÍû¸ß¶È
	set_height_em += (set_speed - hc_value.m_speed) *T;
	set_height_em  = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);
	
	//³¬Éù²¨ºÍÆøÑ¹¼ÆÈÚºÏµÄÊı¾İ·ÖÎö³öÀ´µÄ´¹Ö±ËÙÂÊºÍÆÚÍûµÄÅÊÅÀËÙÂÊ½øĞĞ×ö²î*T=Éè¶¨ÆÚÍû¸ß¶È
	set_height_e += (set_speed - 1.05f *hc_value.fusion_speed) *T;
	set_height_e  = LIMIT(set_height_e,-5000 *ex_i_en,5000 *ex_i_en);
	//ÈÚºÏÁ½¸öÆøÑ¹¼ÆºÍ³¬Éù²¨µÄÊı¾İset_height_e=out
	LPF_1_(0.05f,T,set_height_em,set_height_e);
  //ĞŞ¸ÄÇøÓò
	//set_height_e=0;//mmÎªµ¥Î»£¬ÉèÖÃÆÚÍû¸ß¶È
	if(en<0.1f)
	{
		exp_speed=hc_value.fusion_speed;//ÆÚÍûËÙ¶È=´«¸ĞÆ÷²âÁ¿µÄËÙ¶È
		exp_acc  =hc_value.fusion_acc;  //ÆÚÍû¼ÓËÙ¶È=´«¸ĞÆ÷ÈÚºÏµÄ¼ÓËÙ¶È
	}
  /////////////////////////////////////////////////////////////////////////////////	
	//¼ÓËÙ¶È»ı·ÖÏŞÖÆ·ù¶È
	float acc_i_lim;
	acc_i_lim =safe_div(150,h_acc_arg.ki,0);
	
	fb_speed_old = fb_speed;
	fb_speed = hc_value.fusion_speed;
	fb_acc = safe_div(fb_speed - fb_speed_old,T,0);
	
	//(1)   ¼ÓËÙ¶È»·¿ØÖÆ£¬ÊäÈëÖµÎªÆÚÍû¼ÓËÙ¶È ¿ØÖÆÖÜÆÚ=2ms
	thr_pid_out = PID_calculate( T,       //ÖÜÆÚ
														exp_acc,	  //Ç°À¡
														exp_acc,		//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
														fb_acc,			//·´À¡Öµ
														&h_acc_arg, //PID²ÎÊı½á¹¹Ìå
														&h_acc_val,	//PIDÊı¾İ½á¹¹Ìå
														acc_i_lim*en			//integration limit£¬»ı·ÖÏŞ·ù
														);			//Êä³ö		
	//Æğ·ÉÓÍÃÅ
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
			h_acc_val.err_i += safe_div(150,h_acc_arg.ki,0) *T;
		}
	}
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //Ò»°ë
	
	//ÓÍÃÅ²¹³¥
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45¶ÈÄÚ²¹³¥
	
	//ÓÍÃÅÊä³ö
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );
	
	thr_out = LIMIT(thr_out,0,1000);

/////////////////////////////////////////////////////////////////////////////////	
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{
    //(2)ËÙ¶È»·¿ØÖÆ¿ØÖÆÖÜÆÚ=20ms£¬£¬£¬×¢ÒâÊäÈëµÄÖµÎª £º  ÆÚÍûËÙ¶È+Éè¶¨ËÙ¶È    £¬Éè¶¨ËÙ¶ÈÒ²ÊÇÍ¨¹ıÓÍÃÅÖµÀ´¿ØÖÆµÄ
		exp_acc = PID_calculate(dT,               //ÖÜÆÚ
														exp_speed,				//Ç°À¡
														(set_speed + exp_speed),				//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
														hc_value.fusion_speed,			    //·´À¡Öµ
														&h_speed_arg, //PID²ÎÊı½á¹¹Ìå
														&h_speed_val,	//PIDÊı¾İ½á¹¹Ìå
														500*en			//integration limit£¬»ı·ÖÏŞ·ù
														 );//Êä³ö	
		exp_acc = LIMIT(exp_acc,-3000,3000); 
		
		//------------------------------------¹âÁ÷¶¨µã¿ØÖÆËÙ¶È»·20ms---------------------------------------------------//
		if((PixFlow_TrackLine_flag==1)&&(ultra.height>25)&&(position_control!=0))
		{
			if(position_control==1)//¶¨µã¿ØÖÆ)
			{
					PixFlow_Roll=PID_calculate(dT,      //ÖÜÆÚ
																0,            //Ç°À¡
																exp_speed_x,	//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																X_Speed,			//·´À¡Öµ
																&x_speed_arg, //PID²ÎÊı½á¹¹Ìå
																&x_speed_val,	//PIDÊı¾İ½á¹¹Ìå
																500 *en			  //integration limit£¬»ı·ÖÏŞ·ù
																 );           //Êä³ö	
					PixFlow_Pitch=PID_calculate(dT,     //ÖÜÆÚ
																0,				    //Ç°À¡
																exp_speed_y,  //ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																Y_Speed,			//·´À¡Öµ
																&y_speed_arg, //PID²ÎÊı½á¹¹Ìå
																&y_speed_val,	//PIDÊı¾İ½á¹¹Ìå
																500*en			//integration limit£¬»ı·ÖÏŞ·ù
																 );//Êä³ö
      }
      else if(position_control==2)
			{
			   	PixFlow_Roll=PID_calculate(dT,      //ÖÜÆÚ
																0,            //Ç°À¡
																exp_speed_x,	//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																X_Speed,			//·´À¡Öµ
																&x_speed_arg, //PID²ÎÊı½á¹¹Ìå
																&x_speed_val,	//PIDÊı¾İ½á¹¹Ìå
																500 *en			  //integration limit£¬»ı·ÖÏŞ·ù
																 );           //Êä³ö	
					PixFlow_Pitch=0;
			}			
			PixFlow_Roll=LIMIT((PixFlow_Roll/300.0f),-angle_max,angle_max);
		  PixFlow_Pitch=-LIMIT((PixFlow_Pitch/300.0f),-angle_max,angle_max);
		}
		//------------------------------------¹âÁ÷¶¨µã¿ØÖÆËÙ¶È»·20ms---------------------------------------------------//
		dT2 += dT;
		height_cnt++;
		//(3) ¸ß¶È»·¿ØÖÆÊäÈëÖµÎªÉè¶¨Ä¿±ê¸ß¶È  Éè¶¨Ä¿±ê¸ß¶ÈÊÇÍ¨¹ıÓÍÃÅÖµ¸Ä±äÉè¶¨ËÙ¶È £¬Éè¶¨ËÙ¶ÈºÍÊµ¼ÊËÙ¶È¶ÔÊ±¼ä»ı·ÖµÃµ½Éè¶¨¸ß¶È
		if(height_cnt>=10)//200ms 
		{
		 exp_speed = PID_calculate(dT2,            //ÖÜÆÚ
																0,				//Ç°À¡
																0,				//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																-set_height_e,			//·´À¡Öµ
																&h_height_arg, //PID²ÎÊı½á¹¹Ìå
																&h_height_val,	//PIDÊı¾İ½á¹¹Ìå
																1500 *en			//integration limit£¬»ı·ÖÏŞ·ù
																 );			//Êä³ö	
			
		 exp_speed=LIMIT(exp_speed,-300,300);		
     //------------------------------------¹âÁ÷¶¨µã¿ØÖÆÎ»ÖÃ»·200ms¿ªÊ¼---------------------------------------------------//
	   if((PixFlow_TrackLine_flag==1)&&(ultra.height>25)&&(position_control!=0))
	   {
					 if(position_control==1)//¶¨µã¿ØÖÆ
					 {
							exp_speed_x=PID_calculate(dT2,            //ÖÜÆÚ
																			0,				//Ç°À¡
																			0,				//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																			X_Offset,			//·´À¡Öµ
																			&x_offset_arg, //PID²ÎÊı½á¹¹Ìå
																			&x_offset_val,	//PIDÊı¾İ½á¹¹Ìå
																			1500 *en			//integration limit£¬»ı·ÖÏŞ·ù
																			 );    //Êä³ö	
						
							exp_speed_y=PID_calculate(dT2,            //ÖÜÆÚ
																			0,				//Ç°À¡
																			0,				//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																			Y_Offset,			//·´À¡Öµ
																			&y_offset_arg, //PID²ÎÊı½á¹¹Ìå
																			&y_offset_val,	//PIDÊı¾İ½á¹¹Ìå
																			1500 *en			//integration limit£¬»ı·ÖÏŞ·ù
																			 );			//Êä³ö	
					 }
					 else if(position_control==2)//Ö»½øĞĞºá¹ö¿ØÖÆ
					 {
							exp_speed_x= PID_calculate(dT,            //ÖÜÆÚ
																		0,				//Ç°À¡
																		0,				//ÆÚÍûÖµ£¨Éè¶¨Öµ£©
																		X_Offset,			//·´À¡Öµ
																		&x_offset_arg, //PID²ÎÊı½á¹¹Ìå
																		&x_offset_val,	//PIDÊı¾İ½á¹¹Ìå
																		1500 *en			//integration limit£¬»ı·ÖÏŞ·ù
																		 );    //Êä³ö	
						 exp_speed_y=0;
					 }
					 exp_speed_x = LIMIT(exp_speed_x,-speed_max*100,speed_max*100);//ËÙ¶ÈÏŞÖÆ300
					 exp_speed_y = LIMIT(exp_speed_y,-speed_max*100,speed_max*100);//ËÙ¶ÈÏŞÖÆ300
					}
				  else 
				  {
						exp_speed_x=0;
						exp_speed_y=0;
				  }
					//------------------------------------¹âÁ÷¶¨µã¿ØÖÆÎ»ÖÃ»·200ms½áÊø--------------------------------------------------//
					dT2 = 0;
					height_cnt = 0;
		}
		speed_cnt = 0;
		dT = 0;				
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


