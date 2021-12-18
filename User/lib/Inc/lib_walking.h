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
    float leg_lift_height;  //抬腿高度
    float push_height;
    float body_hight;           //身体高度
    float sinoid_amp;
    float trajectory_centreX;
    float trajectory_centreY;
    float h_offset; //左腿相对于右腿的高度偏差

    float modified_trajectory_centreY;
    float modified_trajectory_centreX;
    float delta_v;
} walkingPara_TypeDef;

typedef struct
{
    float preSwing_x[6];
    float preSwing_y[6];
    float postSwing_x[6];
    float postSwing_y[6];

    float preStance_x[5];
    float preStance_y[5];
    float postStance_x[5];
    float postStance_y[5];

} bezier_control_points_TypeDef;

enum
{
	CALC_PRE_SWING = 0,
	CALC_POST_SWING = 1,
	CALC_PRE_STANCE = 2,
	CALC_POST_STANCE = 3
};

extern bezier_control_points_TypeDef bezier_ctrl_pts;

#include "lib_planning.h"



void walkingPara_struct_init(walkingPara_TypeDef* walkpara, float T_s,float _t,float lambda,float _v,float leg_lift_hight,float body_height);
void troting(float* x_ref,float* y_ref, float time, walkingPara_TypeDef walkpara);
float Interpolate_cubicBezier(float y0, float yf, float x);
float sature(float value,float maxvalue,float minvalue);\
void bezier_planning(float* x_ref,float* y_ref, float time, walkingPara_TypeDef walkpara);
void bezier_sin_planning(float* x_ref,float* y_ref, float* dx_ref, float* dy_ref, \
						float time, walkingPara_TypeDef walkpara);
void calcu_ctrl_pts(int flag, float x0, float y0, float length_step, float h, float push_h);
void calcu_ctrl_pts2(int flag, float x0, float y0, float length_step, float h, float push_h);

#endif
