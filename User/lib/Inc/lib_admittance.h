#ifndef __LIB_ADMITTANCE_H
#define __LIB_ADMITTANCE_H

#include "main.h"
#include "lib_impd.h"

typedef struct __ramp_t
{
    float src;        //输入数据
    float dest;       //输出数据
    float rate;			//速度
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float output;

}ramp_t;

typedef struct __admittance_t
{
	uint8_t init_sign;
	uint8_t init_time;
	float x0;  		//平衡位置
	float dot_x0; 	//平衡点速度
	float xf;  		//修正后变量
	float dot_xf;	//修正后速度
	float ed;  		//修正误差

	float f_ext;   	//外界力
	float dot_ed;  	//修正速度

	float Md; //导纳控制惯性系数
	float Bd; //导纳控制阻尼系数
	float Kd; //导纳控制刚度系数

	float delte_t;  //差分时间
    float deadband; //err < deadband return 0

    float MaxOutput; //输出限幅

    uint32_t process_time_interval;
    uint32_t access_time_now;
    uint32_t access_time_last;

}admittance_t;

extern admittance_t admt_x_diff, admt_y_diff;
extern admittance_t admt_left_x_diff, admt_left_x_diff_2;
extern admittance_t admt_left_y_diff, admt_left_y_diff_2;
extern admittance_t admt_right_x_diff, admt_right_x_diff_2;
extern admittance_t admt_right_y_diff, admt_right_y_diff_2;

void admittance_struct_init(admittance_t* admt, float Kd, float Bd, float MaxOutput, float deadband);
float admittance_calc(admittance_t* admt, float f_ext, float x0); //这个x_f要有初始值，死区内让x_f等于x0
float admittance_calc2(admittance_t* admt, float f_ext, float x0, float dot_x0, uint32_t t_now);
float ramp_calc(ramp_t *rp, float src, float dest);
void admt_param_change(void);
void admit_params_init(void);

#endif
