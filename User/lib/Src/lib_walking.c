/*
 * lib_walking.c
 *
 *  Created on: Sep 11, 2021
 *      Author: haoyun
 */

#include "lib_walking.h"
#include "bsp_xsens.h"
bezier_control_points_TypeDef bezier_ctrl_pts;

#define _IMU_MODIFIED 1

void walkingPara_struct_init(walkingPara_TypeDef* walkpara, float T_s,float _t,float lambda,float _v,float leg_lift_height,float body_height)
{
	walkpara->T_s=T_s;
	walkpara->_t=_t;
	walkpara->lambda=lambda;
	walkpara->_v=_v;
	walkpara->leg_lift_height=leg_lift_height;
	walkpara->body_hight=body_height;

	walkpara->push_height = 16.0;
	walkpara->sinoid_amp = 16.0;

	walkpara->trajectory_centreX[LeftLeg] = 0.0f;
	walkpara->trajectory_centreX[RightLeg] = 0.0f;
	walkpara->record_x[LeftLeg] = 0.0f;
	walkpara->record_x[RightLeg] = 0.0f;
	walkpara->trajectory_centreY = body_height;
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
        *y_ref = Interpolate_cubicBezier(walkpara.body_hight, walkpara.body_hight-walkpara.leg_lift_height, bezier_info*2);
      }
      else
      {
        *y_ref = Interpolate_cubicBezier(walkpara.body_hight-walkpara.leg_lift_height, walkpara.body_hight, bezier_info*2 - 1);
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

//
//void bezier_planning(float* x_ref,float* y_ref, float time, walkingPara_TypeDef walkpara)
//{
//	float distance = walkpara.T_s/2 * walkpara._v;
//	float bezier_info; // bezier_info is the Bezier factor belongs to [0, 1]
//	int gait_phase;
//	float stance_div = 0.5;
//
//
//	if(time>=0 && time < walkpara.T_s*walkpara.lambda) gait_phase = STANCE_PHASE;
//	else if(time>=walkpara.T_s*walkpara.lambda && time <= walkpara.T_s) gait_phase = SWING_PHASE;
//
//	  switch(gait_phase)
//	  {
//	    case STANCE_PHASE:
//	    {
//	    	bezier_info =  time/(walkpara.T_s*walkpara.lambda);
//	    	if(bezier_info < stance_div)
//	    	{
//	    		//bezier/0.5
//	    		calcu_ctrl_pts2(CALC_PRE_STANCE, walkpara.trajectory_centreX, walkpara.trajectory_centreY,
//	    			  	  	  distance, walkpara.leg_lift_height, walkpara.push_height);
//	    		uint16_t dim = sizeof(bezier_ctrl_pts.preStance_x) / sizeof(float);
//	    		float* pts = bezier_generate(bezier_ctrl_pts.preStance_x, bezier_ctrl_pts.preStance_y, dim, bezier_info/stance_div);
//	    		*x_ref = pts[X];
//	    		*y_ref = pts[Y];
//	    	}
//	    	else {
//	    		calcu_ctrl_pts2(CALC_POST_STANCE, walkpara.trajectory_centreX, walkpara.trajectory_centreY,
//	    			    			  	  	  distance, walkpara.leg_lift_height, walkpara.push_height);
//	    		uint16_t dim = sizeof(bezier_ctrl_pts.postStance_x) / sizeof(float);
//
//	    		float* pts = bezier_generate(bezier_ctrl_pts.postStance_x, bezier_ctrl_pts.postStance_y, dim,
//	    				bezier_info/(1.0-stance_div) + stance_div/(stance_div-1));
//	    		*x_ref = pts[X];
//	    		*y_ref = pts[Y];
//			}
//
//	    break;}
//	    case SWING_PHASE:
//	    {
//
//	      bezier_info = time/((1.0-walkpara.lambda)*walkpara.T_s) + walkpara.lambda/(walkpara.lambda-1);
//
//	      if(bezier_info < 0.5)
//	      {
//	    	  calcu_ctrl_pts2(CALC_PRE_SWING, walkpara.trajectory_centreX, walkpara.trajectory_centreY,
//	    			  	  	  distance, walkpara.leg_lift_height, walkpara.push_height);
//	    	  uint16_t dim = sizeof(bezier_ctrl_pts.preSwing_x) / sizeof(float);
//	    	  float* pts = bezier_generate(bezier_ctrl_pts.preSwing_x, bezier_ctrl_pts.preSwing_y, dim, bezier_info*2);
//	    	  *x_ref = pts[X];
//	    	  *y_ref = pts[Y];
//	      }
//	      else
//	      {
//	    	  calcu_ctrl_pts2(CALC_POST_SWING, walkpara.trajectory_centreX, walkpara.trajectory_centreY,
//	    			  	  	  distance, walkpara.leg_lift_height, walkpara.push_height);
//	    	  uint16_t dim = sizeof(bezier_ctrl_pts.postSwing_x) / sizeof(float);
//	    	  float* pts = bezier_generate(bezier_ctrl_pts.postSwing_x, bezier_ctrl_pts.postSwing_y, dim, bezier_info*2 - 1);
//	    	  *x_ref = pts[X];
//	    	  *y_ref = pts[Y];
//
//	      }
//
//
//	    break;}
//	    default: break;
//
//	  }
//
//}

void bezier_sin_planning(int leg, float* x_ref,float* y_ref, float* dx_ref, float* dy_ref, float time)
{
	float distance = robot.walkingParam.T_s/2 * robot.walkingParam._v;
	float bezier_info; // bezier_info is the Bezier factor belongs to [0, 1]
	int gait_phase;
	float temp_x;
	float temp_y;


	if(time>=0 && time < robot.walkingParam.T_s*robot.walkingParam.lambda) gait_phase = STANCE_PHASE;
	else if(time>=robot.walkingParam.T_s*robot.walkingParam.lambda && time <= robot.walkingParam.T_s) gait_phase = SWING_PHASE;

	  switch(gait_phase)
	  {
	    case STANCE_PHASE:
	    {
	    	force_controllor(leg);
	    	posture_controller(xsens_data.pitch);
	    	robot.walkingParam.record_x[leg] = robot.walkingParam.trajectory_centreX[leg];
	    	bezier_info =  time/(robot.walkingParam.T_s*robot.walkingParam.lambda);
	    	temp_x = - distance * bezier_info + robot.walkingParam.trajectory_centreX[leg] + 0.5 * distance;
			temp_y = robot.walkingParam.modified_trajectory_centreY + robot.walkingParam.sinoid_amp * sin(bezier_info * M_PI);
			*dx_ref = - distance / (robot.walkingParam.lambda*robot.walkingParam.T_s);
			*dy_ref = robot.walkingParam.sinoid_amp * cos(bezier_info * M_PI) / (robot.walkingParam.lambda*robot.walkingParam.T_s);

	    break;}
	    case SWING_PHASE:
	    {


	      bezier_info = time/((1.0-robot.walkingParam.lambda)*robot.walkingParam.T_s) + robot.walkingParam.lambda/(robot.walkingParam.lambda-1);
	      //calculate the centreX
	      robot.walkingParam.trajectory_centreX[leg] = - robot.walkingParam.record_x[leg] * bezier_info + robot.walkingParam.record_x[leg];
	      	      if(bezier_info < 0.5)
	      	      {
	      	    	  calcu_ctrl_pts(CALC_PRE_SWING, robot.walkingParam.trajectory_centreX[leg], robot.walkingParam.modified_trajectory_centreY, \
	      	    			  	  	  distance, robot.walkingParam.leg_lift_height, robot.walkingParam.push_height); //calculate the control points
	      	    	  uint16_t dim = sizeof(bezier_ctrl_pts.preSwing_x) / sizeof(float);
	      	    	  float* pts = bezier_generate(bezier_ctrl_pts.preSwing_x, bezier_ctrl_pts.preSwing_y, dim, bezier_info*2);
	      	    	  temp_x = pts[X];
	      	    	  temp_y = pts[Y];
	      	    	  *dx_ref = pts[DX]/((1.0-robot.walkingParam.lambda)*robot.walkingParam.T_s);
	      	    	  *dy_ref = pts[DY]/((1.0-robot.walkingParam.lambda)*robot.walkingParam.T_s);

	      	      }
	      	      else
	      	      {
	      	    	  calcu_ctrl_pts(CALC_POST_SWING, robot.walkingParam.trajectory_centreX[leg], robot.walkingParam.modified_trajectory_centreY, \
	      	    			  	  	  distance, robot.walkingParam.leg_lift_height, robot.walkingParam.push_height);
	      	    	  uint16_t dim = sizeof(bezier_ctrl_pts.postSwing_x) / sizeof(float);
	      	    	  float* pts = bezier_generate(bezier_ctrl_pts.postSwing_x, bezier_ctrl_pts.postSwing_y, dim, bezier_info*2 - 1);
	      	    	  temp_x = pts[X];
	      	    	  temp_y = pts[Y];
	      	    	  *dx_ref = pts[DX]/((1.0-robot.walkingParam.lambda)*robot.walkingParam.T_s);
	      	    	  *dy_ref = pts[DY]/((1.0-robot.walkingParam.lambda)*robot.walkingParam.T_s);

	      	      }

	    break;}
	    default: break;

	  }


#if _IMU_MODIFIED==1
	  *x_ref = temp_x * cos(xsens_data.pitch * M_PI / 180.0f) + temp_y * sin(xsens_data.pitch * M_PI / 180.0f);
	  *y_ref = temp_x * -sin(xsens_data.pitch * M_PI / 180.0f) + temp_y * cos(xsens_data.pitch * M_PI / 180.0f);
#else
	  *x_ref = temp_x;
	  *y_ref = temp_y;
#endif

}

void calcu_ctrl_pts(int flag, float x0, float y0, float length_step, float h, float push_h)
{
	switch (flag) {
		case CALC_PRE_SWING:
		{
			bezier_ctrl_pts.preSwing_x[0] = x0 - 0.5 * length_step;
			bezier_ctrl_pts.preSwing_x[1] = x0 - 0.45 * length_step;
			bezier_ctrl_pts.preSwing_x[2] = x0 - 0.4 * length_step;
			bezier_ctrl_pts.preSwing_x[3] = x0 - 0.3 * length_step;
			bezier_ctrl_pts.preSwing_x[4] = x0 - 0.1 * length_step;
			bezier_ctrl_pts.preSwing_x[5] = x0;

			bezier_ctrl_pts.preSwing_y[0] = y0;
			bezier_ctrl_pts.preSwing_y[1] = y0 - 0.66 * h;
			bezier_ctrl_pts.preSwing_y[2] = y0 - 0.88 * h;
			bezier_ctrl_pts.preSwing_y[3] = y0 - 1.0 * h;
			bezier_ctrl_pts.preSwing_y[4] = y0 - 1.0 * h;
			bezier_ctrl_pts.preSwing_y[5] = y0 - 1.0 * h;

		break;}
		case CALC_POST_SWING:
		{
			bezier_ctrl_pts.postSwing_x[0] = x0;
			bezier_ctrl_pts.postSwing_x[1] = x0 + 0.2 * length_step;
			bezier_ctrl_pts.postSwing_x[2] = x0 + 0.4 * length_step;
			bezier_ctrl_pts.postSwing_x[3] = x0 + 0.5 * length_step;
			bezier_ctrl_pts.postSwing_x[4] = x0 + 0.7 * length_step;
			bezier_ctrl_pts.postSwing_x[5] = x0 + 0.5 * length_step;

			bezier_ctrl_pts.postSwing_y[0] = y0 - 1.0 * h;
			bezier_ctrl_pts.postSwing_y[1] = y0 - 1.0 * h;
			bezier_ctrl_pts.postSwing_y[2] = y0 - 0.8 * h;
			bezier_ctrl_pts.postSwing_y[3] = y0 - 0.6 * h;
			bezier_ctrl_pts.postSwing_y[4] = y0 - 0.2 * h;
			bezier_ctrl_pts.postSwing_y[5] = y0;

		break;}
		case CALC_PRE_STANCE:
		{
			bezier_ctrl_pts.preStance_x[0] = x0 + 0.5 * length_step;
			bezier_ctrl_pts.preStance_x[1] = x0 + 0.34 * length_step;
			bezier_ctrl_pts.preStance_x[2] = x0 + 0.18 * length_step;
			bezier_ctrl_pts.preStance_x[3] = x0 - 0.02 * length_step;
			bezier_ctrl_pts.preStance_x[4] = x0 - 0.14 * length_step;
			bezier_ctrl_pts.preStance_x[5] = x0 - 0.3 * length_step;

			bezier_ctrl_pts.preStance_y[0] = y0;
			bezier_ctrl_pts.preStance_y[1] = y0 + 0.5 * push_h;
			bezier_ctrl_pts.preStance_y[2] = y0;
			bezier_ctrl_pts.preStance_y[3] = y0 - 0.5 * push_h;
			bezier_ctrl_pts.preStance_y[4] = y0 + push_h;
			bezier_ctrl_pts.preStance_y[5] = y0 + push_h;

		break;}
		case CALC_POST_STANCE:
		{
			bezier_ctrl_pts.postStance_x[0] = x0 - 0.3 * length_step;
			bezier_ctrl_pts.postStance_x[1] = x0 - 0.4 * length_step;
			bezier_ctrl_pts.postStance_x[2] = x0 - 0.5 * length_step;

			bezier_ctrl_pts.postStance_y[0] = y0 + push_h;
			bezier_ctrl_pts.postStance_y[1] = y0 + push_h * 0.8;
			bezier_ctrl_pts.postStance_y[2] = y0;

		break;}
		default:
		break;
	}
}

void calcu_ctrl_pts2(int flag, float x0, float y0, float length_step, float h, float push_h)
{
	switch (flag) {
		case CALC_PRE_SWING:
		{
			bezier_ctrl_pts.preSwing_x[0] = x0 - 0.5 * length_step;
			bezier_ctrl_pts.preSwing_x[1] = x0 - 0.55 * length_step;
			bezier_ctrl_pts.preSwing_x[2] = x0 - 0.55 * length_step;
			bezier_ctrl_pts.preSwing_x[3] = x0 - 0.4 * length_step;
			bezier_ctrl_pts.preSwing_x[4] = x0 - 0.2 * length_step;
			bezier_ctrl_pts.preSwing_x[5] = x0;


			bezier_ctrl_pts.preSwing_y[0] = y0;
			bezier_ctrl_pts.preSwing_y[1] = y0;
			bezier_ctrl_pts.preSwing_y[2] = y0 - 0.5 * h;
			bezier_ctrl_pts.preSwing_y[3] = y0 - 0.8 * h;
			bezier_ctrl_pts.preSwing_y[4] = y0 - 1.0 * h;
			bezier_ctrl_pts.preSwing_y[5] = y0 - 1.0 * h;


		break;}
		case CALC_POST_SWING:
		{
			bezier_ctrl_pts.postSwing_x[0] = x0;
			bezier_ctrl_pts.postSwing_x[1] = x0 + 0.2 * length_step;
			bezier_ctrl_pts.postSwing_x[2] = x0 + 0.35 * length_step;
			bezier_ctrl_pts.postSwing_x[3] = x0 + 0.5 * length_step;
			bezier_ctrl_pts.postSwing_x[4] = x0 + 0.55 * length_step;
			bezier_ctrl_pts.postSwing_x[5] = x0 + 0.5 * length_step;


			bezier_ctrl_pts.postSwing_y[0] = y0 - 1.0 * h;
			bezier_ctrl_pts.postSwing_y[1] = y0 - 1.0 * h;
			bezier_ctrl_pts.postSwing_y[2] = y0 - 0.8 * h;
			bezier_ctrl_pts.postSwing_y[3] = y0 - 0.5 * h;
			bezier_ctrl_pts.postSwing_y[4] = y0;
			bezier_ctrl_pts.postSwing_y[5] = y0;


		break;}
		case CALC_PRE_STANCE:
		{
			bezier_ctrl_pts.preStance_x[0] = x0 + 0.5 * length_step;
			bezier_ctrl_pts.preStance_x[1] = x0 + 0.45 * length_step;
			bezier_ctrl_pts.preStance_x[2] = x0 + 0.35 * length_step;
			bezier_ctrl_pts.preStance_x[3] = x0 + 0.2 * length_step;
			bezier_ctrl_pts.preStance_x[4] = x0 + 0.0 * length_step;


			bezier_ctrl_pts.preStance_y[0] = y0;
			bezier_ctrl_pts.preStance_y[1] = y0;
			bezier_ctrl_pts.preStance_y[2] = y0 + 0.5 * push_h;
			bezier_ctrl_pts.preStance_y[3] = y0 + 0.8 * push_h;
			bezier_ctrl_pts.preStance_y[4] = y0 + 1.0 * push_h;


		break;}
		case CALC_POST_STANCE:
		{
			bezier_ctrl_pts.postStance_x[0] = x0 - 0.0 * length_step;
			bezier_ctrl_pts.postStance_x[1] = x0 - 0.2 * length_step;
			bezier_ctrl_pts.postStance_x[2] = x0 - 0.35 * length_step;
			bezier_ctrl_pts.postStance_x[3] = x0 - 0.45 * length_step;
			bezier_ctrl_pts.postStance_x[4] = x0 - 0.5 * length_step;

			bezier_ctrl_pts.postStance_y[0] = y0 + push_h;
			bezier_ctrl_pts.postStance_y[1] = y0 + 0.8 * push_h;
			bezier_ctrl_pts.postStance_y[2] = y0 + 0.5 * push_h;
			bezier_ctrl_pts.postStance_y[3] = y0;
			bezier_ctrl_pts.postStance_y[4] = y0;


		break;}
		default:
		break;
	}
}

