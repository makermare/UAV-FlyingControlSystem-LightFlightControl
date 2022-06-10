#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"

#include "mymath.h"

#include "filter.h"

/*=====================================================================================================================
						Ԥ���㷨
=====================================================================================================================*/
typedef struct
{
	float fb_lim;				 //�����޷�
	float value_old;		 //��ʷֵ
	float value;				 //���������ʵ�ʣ�
	float ut;					   //��Ծ��Ӧ����ʱ��
	float fb_t;					 //�����ͺ�ʱ��
	
	float ct_out;        //�������������
	float feed_back;     //����ֵ
	float fore_feed_back;//���Ƶķ���ֵ
}forecast_t;


void forecast_calculate_I(float T,								//���ڣ���λ���룩
													forecast_t *forecast		//Ԥ���㷨�ṹ��
												 );		

void forecast_calculate_II(float T,								//���ڣ���λ���룩
													 forecast_t *forecast		//Ԥ���㷨�ṹ��
                          );		
													
													
/*=====================================================================================================================
						 *****
=====================================================================================================================*/
typedef struct
{
	float kp;			 //����ϵ��
	float ki;			 //����ϵ��
	float kd;		 	 //΢��ϵ��
	float k_pre_d; //previous_d ΢������
	float inc_hz;  //����ȫ΢�ֵ�ͨϵ��
	float k_inc_d_norm; //Incomplete ����ȫ΢�� ��һ��0,1��
	float k_ff;		 //ǰ�� 
	
}_PID_arg_st;


typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	float err_d;
	float err_d_lpf;
	float err_i;
	float ff;
	float pre_d;

}_PID_val_st;

float PID_calculate( float T,            //����
										float in_ff,				//ǰ��
										float expect,				//����ֵ���趨ֵ��
										float feedback,			//����ֵ
										_PID_arg_st *pid_arg, //PID�����ṹ��
										_PID_val_st *pid_val,	//PID���ݽṹ��
										float inte_lim			//integration limit�������޷�
										   );			//���
										
#endif
