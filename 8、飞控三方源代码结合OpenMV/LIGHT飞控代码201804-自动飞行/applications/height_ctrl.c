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

_hc_value_st hc_value;


u8 thr_take_off_f = 0;
u8 auto_take_off,auto_land;
float height_ref;

float auto_take_off_land(float dT,u8 ready)
{
	static u8    back_home_old;
	static float thr_auto;
	if(ready==0)
	{
		height_ref = hc_value.fusion_height;
		auto_take_off = 0;
	}
	
	if(Thr_Low == 1 && fly_ready == 0)
	{
		if(mode_value[BACK_HOME] == 1 && back_home_old == 0) //���֮ǰ�����ҽ���֮ǰ���Ƿ���ģʽ��������ģʽ
		{
				if(auto_take_off==0)  //��һ�����Զ���ɱ��0->1
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
				if(auto_take_off==2) //�Ѿ������Զ����
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
	
	back_home_old = mode_value[BACK_HOME]; //��¼ģʽ��ʷ
		
	return (thr_auto);
}

_PID_arg_st h_acc_arg;
_PID_arg_st h_speed_arg;
_PID_arg_st h_height_arg;

_PID_val_st h_acc_val;
_PID_val_st h_speed_val;
_PID_val_st h_height_val;

void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//����ϵ��
	h_acc_arg.ki = 0.007f  *pid_setup.groups.hc_sp.kp;				//����ϵ��
	h_acc_arg.kd = 0;				//΢��ϵ��
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;

	h_speed_arg.kp = 5.0f *pid_setup.groups.hc_sp.kp;				//����ϵ��
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//����ϵ��
	h_speed_arg.kd = 0.0f;				//΢��ϵ��
	h_speed_arg.k_pre_d = 0.07f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;	
	
	h_height_arg.kp = 1.0f *pid_setup.groups.hc_height.kp;				//����ϵ��
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//����ϵ��
	h_height_arg.kd = 0.02f *pid_setup.groups.hc_height.kd;				//΢��ϵ��
	h_height_arg.k_pre_d = 0.01f ;
	h_height_arg.inc_hz = 20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;
}



float thr_set,thr_pid_out,thr_out,thr_take_off,tilted_fix;

float en_old;
u8 ex_i_en_f,ex_i_en;


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
	/*�����г��ν��붨��ģʽ�л�����*/
	
	if(ABS(en - en_old) > 0.5f)//�ӷǶ����л�������
	{
		if(thr_take_off<10)//δ�����������
		{
			if(thr_set>-150)
			{
				thr_take_off=400;
			}
		}
		en_old = en;
	}
	if(mode_state==2)//��ѹ��ģʽ��־������һ����ɶ���
	{
			//�޸Ĳ��֣����ȶ���50cm���£�thr=500
			if(ultra.height<=15)
			{
				 thr=650;//����
			}
			else if(ultra.height>15&&ultra.height<20)
			{
			   thr=620;
			}
		 else if(ultra.height>=20&&ultra.height<=25)
		 {
			 thr=580;
		 }
			else if(ultra.height>25&&ultra.height<45)
			{
			   thr=550;
			}
			else if(ultra.height>=45&&ultra.height<=55)
			{
			   thr=500;
			}
			else if(ultra.height>55&&ultra.height<60)
			{
				 thr=450;//�½�
			}
			else if(ultra.height>=60)
			{
			   thr=420;
			} 
	 }
		if(mode_state==1||start_flag==7)//����һ������
		{
		   thr=300;//�½�
		}
		
	//�����趨��ֵ-+500�������ֵ�����趨�����������½�>0������<0 �½�
	thr_set = my_deathzoom_2(my_deathzoom((thr - 500),0,40),0,10);
	
	if(thr_set>0)//��Ӧ����ң�˴���1500����Ҫ����
	{
		//�������������ٶ�
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_UP;//������ٶ�Ϊ��ֵ
		
		if(thr_set>100)//��Ӧ����ң��1640
		{
			ex_i_en_f = 1;//�����־��������ʲô���أ�
			
			if(!thr_take_off_f)//��ɱ�־Ϊ0
			{
				thr_take_off_f = 1; //�û�������Ҫ���
				thr_take_off = 350; //ֱ�Ӹ�ֵ һ��
			}
		}
	}
	
	else//����ң��ֵС��1460����Ҫ�½�
	{
		if(ex_i_en_f == 1)//����������ʹ�ܱ�־
		{
			ex_i_en = 1;//��������ʹ��
		}
		set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;//������ٶ�Ϊ  ��   ֵ
	}
	//���Ƹ߶�������
	set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
	//��ͨ�˲�
	LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);
	//�Ե�ͨ�˲�֮��������ٶȽ������Ʒ���
	set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);
  /////////////////////////////////////////////////////////////////////////////////	
	baro_ctrl(T,&hc_value); //�߶����ݻ�ȡ�� ��ѹ������
  /////////////////////////////////////////////////////////////////////////////////		
	
	//���ﵽ�����߶ȵ�ʱ��  �趨�ٶȵ���0
	
	//����߶����ɼ��˲���
	// MS��ѹ�����ݷ��������Ĵ�ֱ���ʺ��������������ʽ�������*T=�趨�����߶�
	set_height_em += (set_speed - hc_value.m_speed) *T;
	set_height_em  = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);
	
	//����������ѹ���ںϵ����ݷ��������Ĵ�ֱ���ʺ��������������ʽ�������*T=�趨�����߶�
	set_height_e += (set_speed - 1.05f *hc_value.fusion_speed) *T;
	set_height_e = LIMIT(set_height_e,-5000 *ex_i_en,5000 *ex_i_en);
  
	//�ں�������ѹ�ƺͳ�����������set_height_e=out
	LPF_1_(0.05f,T,set_height_em,set_height_e);
	
  //�޸�����
	//set_height_e=500;//mmΪ��λ�����������߶�
	//�޸�����
	
	if(en<0.1f)
	{
		exp_speed=hc_value.fusion_speed;//�����ٶ�=�������������ٶ�
		exp_acc  =hc_value.fusion_acc;  //�������ٶ�=�������ںϵļ��ٶ�
	}
  /////////////////////////////////////////////////////////////////////////////////	
	//���ٶȻ������Ʒ���
	float acc_i_lim;
	acc_i_lim =safe_div(150,h_acc_arg.ki,0);
	
	fb_speed_old = fb_speed;
	fb_speed = hc_value.fusion_speed;
	fb_acc = safe_div(fb_speed - fb_speed_old,T,0);
	
	//(1)   ���ٶȻ����ƣ�����ֵΪ�������ٶ� ��������=2ms
	thr_pid_out = PID_calculate( T,       //����
														exp_acc,	  //ǰ��
														exp_acc,		//����ֵ���趨ֵ��
														fb_acc,			//����ֵ
														&h_acc_arg, //PID�����ṹ��
														&h_acc_val,	//PID���ݽṹ��
														acc_i_lim*en			//integration limit�������޷�
														 );			//���		
	//�������
	if(h_acc_val.err_i > (acc_i_lim * 0.2f))
	{
		if(thr_take_off<THR_TAKE_OFF_LIMIT)
		{
			thr_take_off += 150 *T;
			h_acc_val.err_i -= safe_div(150,h_acc_arg.ki,0)*T;
		}
	}
	else if(h_acc_val.err_i < (-acc_i_lim * 0.2f))
	{
		if(thr_take_off>0)
		{
			thr_take_off -= 150 *T;
			h_acc_val.err_i += safe_div(150,h_acc_arg.ki,0)*T;
		}
	}
	
	thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //һ��
	
	//���Ų���
	tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45���ڲ���
	
	//�������
	thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );
	
	thr_out = LIMIT(thr_out,0,1000);

/////////////////////////////////////////////////////////////////////////////////	
	static float dT,dT2;
	dT += T;
	speed_cnt++;
	if(speed_cnt>=10) //u8  20ms
	{
    //(2)�ٶȻ����ƿ�������=20ms������ע�������ֵΪ ��  �����ٶ�+�趨�ٶ�    ���趨�ٶ�Ҳ��ͨ������ֵ�����Ƶ�
		exp_acc = PID_calculate( dT,              //����
														exp_speed,				//ǰ��
														(set_speed + exp_speed),				//����ֵ���趨ֵ��
														hc_value.fusion_speed,			//����ֵ
														&h_speed_arg, //PID�����ṹ��
														&h_speed_val,	//PID���ݽṹ��
														500 *en			//integration limit�������޷�
														 );			//���	
		
		exp_acc = LIMIT(exp_acc,-3000,3000);
		dT2 += dT;
		height_cnt++;
		//(3) �߶Ȼ�����    ����ֵΪ�趨Ŀ��߶�  �趨Ŀ��߶���ͨ������ֵ�ı��趨�ٶ� ���趨�ٶȺ�ʵ���ٶȶ�ʱ����ֵõ��趨�߶�
		if(height_cnt>=10)  //200ms 
		{
     //�߶Ȼ�����
		 exp_speed = PID_calculate( dT2,            //����
																0,				//ǰ��
																0,				//����ֵ���趨ֵ��
																-set_height_e,			//����ֵ
																&h_height_arg, //PID�����ṹ��
																&h_height_val,	//PID���ݽṹ��
																1500 *en			//integration limit�������޷�
																 );			//���	
			
			exp_speed = LIMIT(exp_speed,-300,300);
			/////////////////////////////////////
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


