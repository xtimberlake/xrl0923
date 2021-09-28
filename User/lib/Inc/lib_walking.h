#ifndef __LIB_WALKING_H
#define __LIB_WALKING_H

#include "main.h"


#define STANCE_PHASE 1
#define SWING_PHASE  0

typedef struct
{
	float T_s;     //步态周期
	float _t;      //当前时间(相位)
	float lambda;  //支撑相占空比
    float _v;       //速度
    float leg_lift_hight;  //抬腿高度
    float body_hight;           //身体高度
} walkingPara_TypeDef;



void walkingPara_struct_init(walkingPara_TypeDef* walkpara, float T_s,float _t,float lambda,float _v,float leg_lift_hight,float body_hight);
void troting(float* x_ref,float* y_ref, float time, walkingPara_TypeDef walkpara);
float Interpolate_cubicBezier(float y0, float yf, float x);
float sature(float value,float maxvalue,float minvalue);

#endif
