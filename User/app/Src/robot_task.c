/*
 * robot_task.c
 *
 *  Created on: Sep 11, 2021
 *      Author: haoyun
 */
#include "robot_task.h"
#include "cmsis_os.h"
#include "bsp_xsens.h"
#include "bsp_6axis.h"

impedance_t impd_left_hip_diff;
impedance_t impd_left_knee_diff;
impedance_t impd_right_hip_diff;
impedance_t impd_right_knee_diff;



pid_pkg pid_robot_height;
pid_pkg pid_kv;
pid_pkg pid_robot_xPosition;


extern osThreadId_t communicatingTaHandle;

void robot_params_init(void)
{
//		impedance_struct_init(&impd_left_hip_diff, 0.2, 0.0, 0.3, 10);
//		impedance_struct_init(&impd_left_knee_diff, 0.2, 0.0, 0.3, 10);
//		impedance_struct_init(&impd_right_hip_diff, 0.2, 0.0, 0.3, 10);
//		impedance_struct_init(&impd_right_knee_diff, 0.2, 0.0, 0.3, 10);

//		admittance_struct_init(&admt_left_hip_diff, 220.0, 100.0, 0.1, 5.0);
//		admittance_struct_init(&admt_left_knee_diff, 220.0, 100.0, 0.1, 5.0);
//		admittance_struct_init(&admt_right_hip_diff, 220.0, 100.0, 0.1, 5.0);	//Kd 越小，返回速度更快
//		admittance_struct_init(&admt_right_knee_diff, 220.0, 100.0, 0.1, 5.0); //Bd减小，越柔顺

		admit_params_init();

		walkingPara_struct_init(&robot.walkingParam, 1.3,0,0.6,0,200,890);

		PID_struct_init(&pid_robot_height, POSITION_PID, 100, 0, 1.0, 0.0, 1.5);
		pid_robot_height.deadband = 1.0f;
		PID_struct_init(&pid_robot_xPosition, POSITION_PID, 100, 0, 0.5, 0.0, 0);
		pid_robot_xPosition.deadband = 3.0f; //float deadline range is 3
		robot.walkingParam.modified_trajectory_centreY = robot.walkingParam.trajectory_centreY;


		PID_struct_init(&pid_kv, POSITION_PID, 10, 0.0, 0.01, 0, 0);

		robot.sensing_t[LAST] = HAL_GetTick();
		robot.sensing_t[NOW] = HAL_GetTick();

		robot.leftLeg.last_x = robot.leftLeg.x;
		robot.leftLeg.last_y = robot.leftLeg.y;
		robot.rightLeg.last_x = robot.rightLeg.x;
		robot.rightLeg.last_y = robot.rightLeg.y;
		osDelay(1000);
		force.defaultForce = force.FZ1;
		force.k = 0.01;
}

void robot_task(void *argument)
{
  /* USER CODE BEGIN robot_task */

	robot_params_init();
  /* Infinite loop */
  for(;;)
  {
	motor_param_change();
	admt_param_change();
	//robot_sensing();
	switch(robot.state)
	{
		case INITIALIZING:
		{
			robot.leftLeg.x_ref = robot.leftLeg.x;
			robot.leftLeg.y_ref = robot.leftLeg.y;
			robot.rightLeg.x_ref = robot.rightLeg.x;
			robot.rightLeg.y_ref = robot.rightLeg.y;
			break;
		}
		case WORKING:
		{
			robot.walkingParam.trajectory_centreX[LeftLeg] = 0;
			robot.walkingParam.trajectory_centreX[RightLeg] = 0;
			robot.leftLeg.x_ref = robot.walkingParam.trajectory_centreX[LeftLeg];
			robot.leftLeg.y_ref = robot.walkingParam.trajectory_centreY;
			robot.rightLeg.x_ref = robot.walkingParam.trajectory_centreX[RightLeg];
			robot.rightLeg.y_ref = robot.walkingParam.trajectory_centreY;
			robot.leftLeg.dx_ref = 0;
			robot.leftLeg.dy_ref = 0;
			robot.rightLeg.dx_ref = 0;
			robot.rightLeg.dy_ref = 0;

			robot.leftLeg.y_ref = robot.walkingParam.modified_trajectory_centreY + robot.walkingParam.h_offset;
			robot.rightLeg.y_ref = robot.walkingParam.modified_trajectory_centreY;
			robot_acting();
			break;
		}
		case YADJUST:
		{
			robot.leftLeg.x_ref = robot.walkingParam.trajectory_centreX[LeftLeg];
			robot.leftLeg.y_ref = robot.walkingParam.trajectory_centreY;
			robot.rightLeg.x_ref = robot.walkingParam.trajectory_centreX[RightLeg];
			robot.rightLeg.y_ref = robot.walkingParam.trajectory_centreY;
			robot.leftLeg.dx_ref = 0;
			robot.leftLeg.dy_ref = 0;
			robot.rightLeg.dx_ref = 0;
			robot.rightLeg.dy_ref = 0;

			posture_controller(xsens_data.pitch);
//			force_controllor(force.FZ1);

			robot.leftLeg.y_ref = robot.walkingParam.modified_trajectory_centreY + robot.walkingParam.h_offset;
			robot.rightLeg.y_ref = robot.walkingParam.modified_trajectory_centreY;
			robot_acting();
			break;
		}
		case XADJUST:
		{
//			robot.leftLeg.x_ref = robot.walkingParam.trajectory_centreX;
//			robot.leftLeg.y_ref = robot.walkingParam.trajectory_centreY;
//			robot.rightLeg.x_ref = robot.walkingParam.trajectory_centreX;
//			robot.rightLeg.y_ref = robot.walkingParam.trajectory_centreY;
//			robot.leftLeg.dx_ref = 0;
//			robot.leftLeg.dy_ref = 0;
//			robot.rightLeg.dx_ref = 0;
//			robot.rightLeg.dy_ref = 0;
//
//			//posture_controller(xsens_data.pitch);
//			force_controllor();
//			robot.leftLeg.x_ref = robot.walkingParam.modified_trajectory_centreX;
//			robot.rightLeg.x_ref = robot.walkingParam.modified_trajectory_centreX;
//			robot.leftLeg.y_ref = robot.walkingParam.modified_trajectory_centreY + robot.walkingParam.h_offset;
//			robot.rightLeg.y_ref = robot.walkingParam.modified_trajectory_centreY;
//			robot_acting();
			break;
		}
		case BOTHXY:
		{
//			robot.leftLeg.x_ref = robot.walkingParam.trajectory_centreX;
//			robot.leftLeg.y_ref = robot.walkingParam.trajectory_centreY;
//			robot.rightLeg.x_ref = robot.walkingParam.trajectory_centreX;
//			robot.rightLeg.y_ref = robot.walkingParam.trajectory_centreY;
//			robot.leftLeg.dx_ref = 0;
//			robot.leftLeg.dy_ref = 0;
//			robot.rightLeg.dx_ref = 0;
//			robot.rightLeg.dy_ref = 0;
//
//			posture_controller(xsens_data.pitch);
//			force_controllor();
//			robot.leftLeg.x_ref = robot.walkingParam.modified_trajectory_centreX;
//			robot.rightLeg.x_ref = robot.walkingParam.modified_trajectory_centreX;
//			robot.leftLeg.y_ref = robot.walkingParam.modified_trajectory_centreY + robot.walkingParam.h_offset;
//			robot.rightLeg.y_ref = robot.walkingParam.modified_trajectory_centreY;
//			robot_acting();
			break;
		}
		case TROTING:
		{
//			troting(&robot.leftLeg.x_ref,&robot.leftLeg.y_ref, robot.walkingParam._t, robot.walkingParam);
//			if(robot.walkingParam._t<=robot.walkingParam.T_s/2){
//				troting(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref, robot.walkingParam._t+robot.walkingParam.T_s/2, robot.walkingParam);
//			}
//			else{
//				troting(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref, robot.walkingParam._t-robot.walkingParam.T_s/2, robot.walkingParam);
//			}

//			bezier_planning(&robot.leftLeg.x_ref,&robot.leftLeg.y_ref, robot.walkingParam._t, robot.walkingParam);
//			if(robot.walkingParam._t<=robot.walkingParam.T_s/2){
//				bezier_planning(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref, robot.walkingParam._t + robot.walkingParam.T_s/2,  robot.walkingParam);
//			}
//			else{
//				bezier_planning(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref, robot.walkingParam._t-robot.walkingParam.T_s/2, robot.walkingParam);
//			}

//			posture_controller(xsens_data.pitch);
			bezier_sin_planning(LeftLeg, &robot.leftLeg.x_ref,&robot.leftLeg.y_ref, &robot.leftLeg.dx_ref, &robot.leftLeg.dy_ref\
							,robot.walkingParam._t);

			if(robot.walkingParam._t<=robot.walkingParam.T_s/2){
				bezier_sin_planning(RightLeg, &robot.rightLeg.x_ref,&robot.rightLeg.y_ref, &robot.rightLeg.dx_ref, &robot.rightLeg.dy_ref\
							,robot.walkingParam._t + robot.walkingParam.T_s/2);
			}
			else{
				bezier_sin_planning(RightLeg, &robot.rightLeg.x_ref,&robot.rightLeg.y_ref, &robot.rightLeg.dx_ref, &robot.rightLeg.dy_ref\
							,robot.walkingParam._t-robot.walkingParam.T_s/2);
			} // keep the right leg move behind the left half of the cycle

//			bezier_planning(&robot.leftLeg.x_ref,&robot.leftLeg.y_ref
//							,robot.walkingParam._t, robot.walkingParam);
//
//			if(robot.walkingParam._t<=robot.walkingParam.T_s/2){
//				bezier_planning(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref
//							,robot.walkingParam._t + robot.walkingParam.T_s/2,  robot.walkingParam);
//			}
//			else{
//				bezier_planning(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref
//							,robot.walkingParam._t-robot.walkingParam.T_s/2, robot.walkingParam);
//			}

			robot.leftLeg.y_ref += robot.walkingParam.h_offset;

			if(robot.walkingParam._t>=robot.walkingParam.T_s){
				robot.walkingParam._t=0;
			}
			robot.walkingParam._t+=0.005;



			robot_acting();
			break;
		}
	}
	osDelay(2); //5-3
  }
  /* USER CODE END robot_task */
}



void robot_acting()
{
//	uint32_t t_now;
//	t_now = HAL_GetTick();
//
//	admt_foot_point_control2(t_now);
//	// Left X:
//	admittance_calc2(&admt_left_x_diff, -robot.leftLeg.fx, robot.leftLeg.x_ref, robot.leftLeg.dx, t_now);
//	robot.leftLeg.ad_x_ref = robot.leftLeg.x_ref - admt_left_x_diff.ed;
//	// Left Y:
//	admittance_calc2(&admt_left_y_diff, -robot.leftLeg.fy, robot.leftLeg.y_ref, robot.leftLeg.dy, t_now);
//	robot.leftLeg.ad_y_ref = robot.leftLeg.y_ref - admt_left_y_diff.ed;
//
//	// Right X:
//	admittance_calc2(&admt_right_x_diff, -robot.rightLeg.fx, robot.rightLeg.x_ref, robot.rightLeg.dx, t_now);
//	robot.rightLeg.ad_x_ref = robot.rightLeg.x_ref - admt_right_x_diff.ed;
//	// Right Y:
//	admittance_calc2(&admt_right_y_diff, -robot.rightLeg.fy, robot.rightLeg.y_ref, robot.rightLeg.dy, t_now);
//	robot.rightLeg.ad_y_ref = robot.rightLeg.y_ref - admt_right_y_diff.ed;

	normal_control();

	//robot_impedance_ctrller();

//	robot_admittance_ctrller();
//
//
//	leftHip_Motor.ref_cmd = sature(leftHip_Motor.ref_cmd, RIGHT_HIP_MAX_ANGLE, RIGHT_HIP_MIN_ANGLE);
//	tmotor_set_position(LEFT_HIP_MOTOR_ID, leftHip_Motor.ref_cmd, leftHip_Motor.Kp_theta);
//	//osDelay(1);
//	leftKnee_Motor.ref_cmd = sature(leftKnee_Motor.ref_cmd, RIGHT_KNEE_MAX_ANGLE, RIGHT_KNEE_MIN_ANGLE);
//	tmotor_set_position(LEFT_KNEE_MOTOR_ID, leftKnee_Motor.ref_cmd, leftKnee_Motor.Kp_theta);
//
//	//osDelay(1);
//	rightHip_Motor.ref_cmd = sature(rightHip_Motor.ref_cmd, LEFT_HIP_MAX_ANGLE, LEFT_HIP_MIN_ANGLE);
//	tmotor_set_position(RIGHT_HIP_MOTOR_ID, rightHip_Motor.ref_cmd, rightHip_Motor.Kp_theta);
//	//osDelay(1);
//	rightKnee_Motor.ref_cmd = sature(rightKnee_Motor.ref_cmd, LEFT_KNEE_MAX_ANGLE, LEFT_KNEE_MIN_ANGLE);
//	tmotor_set_position(RIGHT_KNEE_MOTOR_ID, rightKnee_Motor.ref_cmd, rightKnee_Motor.Kp_theta);

#if USE_UNITREE == 1
	// do some sature and calculation here
	set_send_signal();

#else
	leftHip_Motor.ref_position = sature(leftHip_Motor.ref_position, RIGHT_HIP_MAX_ANGLE, RIGHT_HIP_MIN_ANGLE);
	tmotor_set_position(LEFT_HIP_MOTOR_ID, leftHip_Motor.ref_position, leftHip_Motor.Kp_theta);
	osDelay(1);
	leftKnee_Motor.ref_position = sature(leftKnee_Motor.ref_position, RIGHT_KNEE_MAX_ANGLE, RIGHT_KNEE_MIN_ANGLE);
	tmotor_set_position(LEFT_KNEE_MOTOR_ID, leftKnee_Motor.ref_position, leftKnee_Motor.Kp_theta);
	osDelay(1);

	rightHip_Motor.ref_position = sature(rightHip_Motor.ref_position, LEFT_HIP_MAX_ANGLE, LEFT_HIP_MIN_ANGLE);
	tmotor_set_position(RIGHT_HIP_MOTOR_ID, rightHip_Motor.ref_position, rightHip_Motor.Kp_theta);
	osDelay(1);
	rightKnee_Motor.ref_position = sature(rightKnee_Motor.ref_position, LEFT_KNEE_MAX_ANGLE, LEFT_KNEE_MIN_ANGLE);
	tmotor_set_position(RIGHT_KNEE_MOTOR_ID, rightKnee_Motor.ref_position, rightKnee_Motor.Kp_theta);
#endif
}

void normal_control(void)
{
	float phi_left_1, phi_left_2;
	float phi_right_1, phi_right_2;
	//电机角度补偿计算
	phi_left_1 = leftHip_Motor.assemble_bias - leftHip_Motor.curr_position;
	phi_left_2 = leftKnee_Motor.assemble_bias - leftKnee_Motor.curr_position;
	phi_right_1 = rightHip_Motor.curr_position + rightHip_Motor.assemble_bias;
	phi_right_2 = rightKnee_Motor.curr_position + rightKnee_Motor.assemble_bias;
	//robot_leg_admittance_ctrller();

	//注意调用完左腿ik后立即使用ang_left,否则会被右腿ik覆盖
	float* ang_left = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.leftLeg.x_ref, robot.leftLeg.y_ref);
	leftHip_Motor.ref_position = -ang_left[0] + leftHip_Motor.assemble_bias;
	leftKnee_Motor.ref_position = -ang_left[1] + leftKnee_Motor.assemble_bias ;

	float* dq_left = calcu_dq(phi_left_1 + M_PI_2, phi_left_1 + phi_left_2, robot.leftLeg.dx_ref, robot.leftLeg.dy_ref, \
			LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	leftHip_Motor.ref_velocity = dq_left[0];
	leftKnee_Motor.ref_velocity = dq_left[1];

	float* ang_right = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.rightLeg.x_ref, robot.rightLeg.y_ref);
	rightHip_Motor.ref_position = ang_right[0] - rightHip_Motor.assemble_bias;
	rightKnee_Motor.ref_position = ang_right[1] - rightKnee_Motor.assemble_bias;
	float* dq_right = calcu_dq(phi_right_1 + M_PI_2, phi_right_1 + phi_right_2, robot.rightLeg.dx_ref, robot.rightLeg.dy_ref, \
			LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	rightHip_Motor.ref_velocity = dq_right[0];
	rightKnee_Motor.ref_velocity = dq_right[1];
}

void admt_foot_point_control()
{
	float phi_left_1, phi_left_2;
	float phi_right_1, phi_right_2;

	// IK Position:
	// Left X:
	admittance_calc(&admt_left_x_diff, -robot.leftLeg.fx, robot.leftLeg.x_ref);
	robot.leftLeg.ad_x_ref = robot.leftLeg.x_ref - admt_left_x_diff.ed;
	// Left Y:
	admittance_calc(&admt_left_y_diff, -robot.leftLeg.fy, robot.leftLeg.y_ref);
	robot.leftLeg.ad_y_ref = robot.leftLeg.y_ref - admt_left_y_diff.ed;

	float* ang_left = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.leftLeg.ad_x_ref, robot.leftLeg.ad_y_ref);
	leftHip_Motor.ref_position = -ang_left[0] + leftHip_Motor.assemble_bias;
	leftKnee_Motor.ref_position = -ang_left[1] + leftKnee_Motor.assemble_bias ;

	// Right X:
	admittance_calc(&admt_right_x_diff, -robot.rightLeg.fx, robot.rightLeg.x_ref);
	robot.rightLeg.ad_x_ref = robot.rightLeg.x_ref - admt_right_x_diff.ed;
	// Right Y:
	admittance_calc(&admt_right_y_diff, -robot.rightLeg.fy, robot.rightLeg.y_ref);
	robot.rightLeg.ad_y_ref = robot.rightLeg.y_ref - admt_right_y_diff.ed;

	float* ang_right = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.rightLeg.ad_x_ref, robot.rightLeg.ad_y_ref);
	rightHip_Motor.ref_position = ang_right[0] - rightHip_Motor.assemble_bias;
	rightKnee_Motor.ref_position = ang_right[1] - rightKnee_Motor.assemble_bias;




	// IK Velocity
	phi_left_1 = leftHip_Motor.assemble_bias - leftHip_Motor.curr_position;
	phi_left_2 = leftKnee_Motor.assemble_bias - leftKnee_Motor.curr_position;
	phi_right_1 = rightHip_Motor.curr_position + rightHip_Motor.assemble_bias;
	phi_right_2 = rightKnee_Motor.curr_position + rightKnee_Motor.assemble_bias;

	float* dq_left = calcu_dq(phi_left_1 + M_PI_2, phi_left_1 + phi_left_2, robot.leftLeg.ad_x_ref, robot.leftLeg.ad_y_ref, \
				LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	leftHip_Motor.ref_velocity = dq_left[0];
	leftKnee_Motor.ref_velocity = dq_left[1];

	float* dq_right = calcu_dq(phi_right_1 + M_PI_2, phi_right_1 + phi_right_2, robot.rightLeg.ad_x_ref, robot.rightLeg.ad_y_ref, \
			LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	rightHip_Motor.ref_velocity = dq_right[0];
	rightKnee_Motor.ref_velocity = dq_right[1];

}
void admt_foot_point_control2(uint32_t t)
{
	float phi_left_1, phi_left_2;
	float phi_right_1, phi_right_2;

	// IK Position:
	// Left X:
	admittance_calc2(&admt_left_x_diff_2, -robot.leftLeg.fx, robot.leftLeg.x_ref, robot.leftLeg.dx, t);
	robot.leftLeg.ad_x_ref = robot.leftLeg.x_ref - admt_left_x_diff.ed;
	// Left Y:
	admittance_calc2(&admt_left_y_diff_2, -robot.leftLeg.fy, robot.leftLeg.y_ref, robot.leftLeg.dy, t);
	robot.leftLeg.ad_y_ref = robot.leftLeg.y_ref - admt_left_y_diff.ed;

	float* ang_left = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.leftLeg.ad_x_ref, robot.leftLeg.ad_y_ref);
	leftHip_Motor.ref_position = -ang_left[0] + leftHip_Motor.assemble_bias;
	leftKnee_Motor.ref_position = -ang_left[1] + leftKnee_Motor.assemble_bias ;

	// Right X:
	admittance_calc2(&admt_right_x_diff_2, -robot.rightLeg.fx, robot.rightLeg.x_ref, robot.rightLeg.dx, t);
	robot.rightLeg.ad_x_ref = robot.rightLeg.x_ref - admt_right_x_diff.ed;
	// Right Y:
	admittance_calc2(&admt_right_y_diff_2, -robot.rightLeg.fy, robot.rightLeg.y_ref, robot.rightLeg.dy, t);
	robot.rightLeg.ad_y_ref = robot.rightLeg.y_ref - admt_right_y_diff.ed;

	float* ang_right = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.rightLeg.ad_x_ref, robot.rightLeg.ad_y_ref);
	rightHip_Motor.ref_position = ang_right[0] - rightHip_Motor.assemble_bias;
	rightKnee_Motor.ref_position = ang_right[1] - rightKnee_Motor.assemble_bias;




	// IK Velocity
	phi_left_1 = leftHip_Motor.assemble_bias - leftHip_Motor.curr_position;
	phi_left_2 = leftKnee_Motor.assemble_bias - leftKnee_Motor.curr_position;
	phi_right_1 = rightHip_Motor.curr_position + rightHip_Motor.assemble_bias;
	phi_right_2 = rightKnee_Motor.curr_position + rightKnee_Motor.assemble_bias;

	float* dq_left = calcu_dq(phi_left_1 + M_PI_2, phi_left_1 + phi_left_2, robot.leftLeg.ad_x_ref, robot.leftLeg.ad_y_ref, \
				LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	leftHip_Motor.ref_velocity = dq_left[0];
	leftKnee_Motor.ref_velocity = dq_left[1];

	float* dq_right = calcu_dq(phi_right_1 + M_PI_2, phi_right_1 + phi_right_2, robot.rightLeg.ad_x_ref, robot.rightLeg.ad_y_ref, \
			LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	rightHip_Motor.ref_velocity = dq_right[0];
	rightKnee_Motor.ref_velocity = dq_right[1];

}

void robot_impedance_ctrller(void)
{
		impedance_calc(&impd_left_hip_diff, 0, leftHip_Motor.curr_torque, 0, 0); //如果力矩是正数，impd——hip输出负数
		impedance_calc(&impd_left_knee_diff, 0, leftHip_Motor.curr_torque, 0, 0);

		//右边向上拉，两个电机的扭矩都会减小
		impedance_calc(&impd_right_hip_diff, 0, rightHip_Motor.curr_torque, 0, 0); //如果力矩是负数，impd——hip输出正数
		impedance_calc(&impd_right_knee_diff, 0, rightKnee_Motor.curr_torque, 0, 0);

		leftHip_Motor.ref_position += impd_left_hip_diff.pos_out;
		leftKnee_Motor.ref_position += impd_left_knee_diff.pos_out;

		rightHip_Motor.ref_position += impd_right_hip_diff.pos_out;
		rightKnee_Motor.ref_position += impd_right_knee_diff.pos_out;
}

//void robot_admittance_ctrller(void)
//{
//	admittance_calc(&admt_right_hip_diff, rightHip_Motor.average_torque, rightHip_Motor.ref_position);
//	rightHip_Motor.ref_cmd = rightHip_Motor.ref_position - admt_right_hip_diff.ed;
//
//	//rightHip_Motor.ref_cmd = rightHip_Motor.ref_position;
//
//	rightKnee_Motor.ref_cmd = rightKnee_Motor.ref_position;
////	admittance_calc(&admt_right_knee_diff, rightKnee_Motor.average_torque, rightKnee_Motor.ref_position);
////	rightKnee_Motor.ref_cmd = rightKnee_Motor.ref_position - admt_right_knee_diff.ed;
//
//	admittance_calc(&admt_left_hip_diff, leftHip_Motor.average_torque, leftHip_Motor.ref_position);
//	leftHip_Motor.ref_cmd = leftHip_Motor.ref_position - admt_left_hip_diff.ed;
//	//leftHip_Motor.ref_cmd = leftHip_Motor.ref_position;
//
//	leftKnee_Motor.ref_cmd = leftKnee_Motor.ref_position;
////	admittance_calc(&admt_left_knee_diff, leftKnee_Motor.average_torque, leftKnee_Motor.ref_position);
////	leftKnee_Motor.ref_cmd = leftKnee_Motor.ref_position - admt_left_knee_diff.ed;
//
//}
//void robot_leg_admittance_ctrller(void)
//{
//	admittance_calc(&admt_left_x_diff, robot.leftLeg.fx, robot.leftLeg.x_ref);
//	robot.leftLeg.x_cmd = robot.leftLeg.x_ref-admt_left_x_diff.ed;
//
//	admittance_calc(&admt_left_y_diff, robot.leftLeg.fy, robot.leftLeg.y_ref);
//	robot.leftLeg.y_cmd = robot.leftLeg.y_ref-admt_left_y_diff.ed;
//
//	admittance_calc(&admt_right_x_diff, robot.rightLeg.fx, robot.rightLeg.x_ref);
//	robot.rightLeg.x_cmd = robot.rightLeg.x_ref-admt_right_x_diff.ed;
//
//	admittance_calc(&admt_right_y_diff, robot.rightLeg.fy, robot.rightLeg.y_ref);
//	robot.rightLeg.y_cmd = robot.rightLeg.y_ref-admt_right_y_diff.ed;
//}

void posture_controller(float theta)
{
	//dy = PID(-theta)
	// acutal_trajactory_center_y =
	robot.walkingParam.modified_trajectory_centreY += 0.01 * pid_calc(&pid_robot_height, theta, 0.0);
//	robot.walkingParam.modified_trajectory_centreY  = sature(robot.walkingParam.modified_trajectory_centreY ,
//			1045-robot.walkingParam.sinoid_amp, 895+robot.walkingParam.leg_lift_height);
	robot.walkingParam.modified_trajectory_centreY  = sature(robot.walkingParam.modified_trajectory_centreY , \
				950, 720);
}
float mid_data;
//float default_value = -80.0f;
void force_controllor(int leg)
{

	static int last_leg = 0;
	if(leg != last_leg)
	{
		pid_robot_xPosition.err[LAST] = 0;
		last_leg = leg;
	}

//	-------------------------Version1-----------------------------------
	mid_data = pid_calc(&pid_robot_xPosition, force.FZ1, force.defaultForce);

	robot.walkingParam.trajectory_centreX[leg] += 0.01*mid_data;
	robot.walkingParam.trajectory_centreX[leg] = sature(robot.walkingParam.trajectory_centreX[leg] ,300, -300);



	force.deltaF = force.FZ1-force.defaultForce;
	robot.walkingParam.delta_v = force.deltaF*force.k;
	robot.walkingParam._v += robot.walkingParam.delta_v;
	robot.walkingParam._v = sature(robot.walkingParam._v ,450, 0.0f);

}


