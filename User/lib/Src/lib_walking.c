/*
 * lib_walking.c
 *
 *  Created on: Sep 11, 2021
 *      Author: haoyun
 */

#include "lib_walking.h"

void walkingPara_struct_init(walkingPara_TypeDef* walkpara, float T_s,float _t,float lambda,float _v,float leg_lift_hight,float body_hight)
{
	walkpara->T_s=T_s;
	walkpara->_t=_t;
	walkpara->lambda=lambda;
	walkpara->_v=_v;
	walkpara->leg_lift_hight=leg_lift_hight;
	walkpara->body_hight=body_hight;
}


void troting(float* x_ref,float* y_ref, float time, walkingPara_TypeDef walkpara)
{
  float p0,pf;
  float distance = walkpara.T_s/2 * walkpara._v;
  float bezier_info;
  int gait_phase;
  p0 = -distance/2;
  pf = distance/2;

  if(time>=0 && time < walkpara.T_s*walkpara.lambda) gait_phase = STANCE_PHASE;
  else if(time>=walkpara.T_s*walkpara.lambda && time <= walkpara.T_s) gait_phase = SWING_PHASE;

  switch(gait_phase)
  {
    case STANCE_PHASE:
    {
      bezier_info =  time/(walkpara.T_s*walkpara.lambda);
      *x_ref = Interpolate_cubicBezier(pf, p0, bezier_info);
      *y_ref = walkpara.body_hight;

    break;}
    case SWING_PHASE:
    {
      bezier_info = time/((1.0-walkpara.lambda)*walkpara.T_s) + walkpara.lambda/(walkpara.lambda-1);

      *x_ref  = Interpolate_cubicBezier(p0, pf, bezier_info);

      if(bezier_info < 0.5)
      {
        *y_ref = Interpolate_cubicBezier(walkpara.body_hight, walkpara.body_hight-walkpara.leg_lift_hight, bezier_info*2);
      }
      else
      {
        *y_ref = Interpolate_cubicBezier(walkpara.body_hight-walkpara.leg_lift_hight, walkpara.body_hight, bezier_info*2 - 1);
      }


    break;}
    default: break;

  }
  //printf("x_ref: %.2f  z_ref:%.2f\n", robot.leftLeg.x_ref , robot.leftLeg.z_ref);
}

float Interpolate_cubicBezier(float y0, float yf, float x) {

 x = sature(x,1,0);
 float yDiff = yf - y0;
 float bezier = x * x * x + 3.0 * (x * x * (1.0 - x));
 return y0 + bezier * yDiff;
}

float sature(float value,float maxvalue,float minvalue){
	if(value<minvalue){
		value=minvalue;
	}
	else if(value>maxvalue){
		value=maxvalue;
	}
	return value;

}
