/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_PID.c
 * 描述    ：PID函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "Ano_Pid.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"


float PID_calculate( float dT_s,            //周期（单位：秒）
                     float in_ff,				//前馈值
                     float expect,				//期望值（设定值）
                     float feedback,			//反馈值（）
                     _PID_arg_st *pid_arg, //PID参数结构体
                     _PID_val_st *pid_val,	//PID数据结构体
                     float inte_d_lim,//积分误差限幅
                     float inte_lim			//integration limit，积分限幅
                   )
{
  float differential,hz;
  hz = safe_div(1.0f,dT_s,0);

	//计算期望值微分
  pid_val->exp_d = (expect - pid_val->exp_old) *hz;

	//计算反馈值微分
  if(pid_arg->fb_d_mode == 0) {
    pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;
  } else {
    pid_val->fb_d = pid_val->fb_d_ex;
  }
	
	//计算输出微分项
  differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);

	//计算误差值
  pid_val->err = (expect - feedback);
 
	//计算误差积分值
  pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;
	
	//误差积分限幅
  pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);


  pid_val->out = pid_arg->k_ff *in_ff
                 + pid_arg->kp *pid_val->err
                 +	differential
                 + pid_val->err_i;

	
	//保存上次反馈
  pid_val->feedback_old = feedback;
	//保存上次期望
  pid_val->exp_old = expect;

  return (pid_val->out);
}





/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/


