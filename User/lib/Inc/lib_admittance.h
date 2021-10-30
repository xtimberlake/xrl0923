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
	float x0;  //修正前变量
	float dx0; //修正前速度
	float xf;  //修正后变量
	float ed;  //修正误差

	float f_ext;   //外界力
	float dot_ed;  //输出目标误差导数

	float Md; //导纳控制惯性系数
	float Bd; //导纳控制阻尼系数
	float Kd; //导纳控制刚度系数

	float delte_t;  //差分时间
    float deadband; //err < deadband return 0

    float MaxOutput; //输出限幅

}admittance_t;

void admittance_struct_init(admittance_t* admt, float Kd, float Bd, float MaxOutput, float deadband);
float admittance_calc(admittance_t* admt, float f_ext, float x0, float dx0); //这个x_f要有初始值，死区内让x_f等于x0
float ramp_calc(ramp_t *rp, float src, float dest);

#endif
