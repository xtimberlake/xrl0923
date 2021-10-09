/*
 * robot_task.c
 *
 *  Created on: Sep 11, 2021
 *      Author: haoyun
 */
#include "robot_task.h"

impedance_t impd_left_hip_diff;
impedance_t impd_left_knee_diff;
impedance_t impd_right_hip_diff;
impedance_t impd_right_knee_diff;

admittance_t admt_left_hip_diff;
admittance_t admt_left_knee_diff;
admittance_t admt_right_hip_diff;
admittance_t admt_right_knee_diff;

admittance_t admt_left_x_diff;
admittance_t admt_left_y_diff;
admittance_t admt_right_x_diff;
admittance_t admt_right_y_diff;

void robot_task(void *argument)
{
  /* USER CODE BEGIN robot_task */
	impedance_struct_init(&impd_left_hip_diff, 0.2, 0.0, 0.3, 10);
	impedance_struct_init(&impd_left_knee_diff, 0.2, 0.0, 0.3, 10);
	impedance_struct_init(&impd_right_hip_diff, 0.2, 0.0, 0.3, 10);
	impedance_struct_init(&impd_right_knee_diff, 0.2, 0.0, 0.3, 10);

	admittance_struct_init(&admt_left_hip_diff, 220.0, 100.0, 0.1, 5.0);
	admittance_struct_init(&admt_left_knee_diff, 220.0, 100.0, 0.1, 5.0);
	admittance_struct_init(&admt_right_hip_diff, 220.0, 100.0, 0.1, 5.0);	//Kd 越小，返回速度更快
	admittance_struct_init(&admt_right_knee_diff, 220.0, 100.0, 0.1, 5.0); //Bd减小，越柔顺

	admittance_struct_init(&admt_left_x_diff, 0.2, 4, 150.0, 5.0);
	admittance_struct_init(&admt_left_y_diff, 0.2, 4, 150.0, 5.0);
	admittance_struct_init(&admt_right_x_diff, 0.2, 4, 150.0, 5.0);
	admittance_struct_init(&admt_right_y_diff, 0.2, 4, 150.0, 5.0);

	walkingPara_struct_init(&robot.walkingParam, 1.0,0,0.7,0,150,900);

  /* Infinite loop */
  for(;;)
  {
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
			robot.leftLeg.x_ref =0;
			robot.leftLeg.y_ref = 900;
			robot.rightLeg.x_ref = 0;
			robot.rightLeg.y_ref = 900;
			robot_acting();
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

			bezier_planning(&robot.leftLeg.x_ref,&robot.leftLeg.y_ref, robot.walkingParam._t, robot.walkingParam);
			if(robot.walkingParam._t<=robot.walkingParam.T_s/2){
				bezier_planning(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref, robot.walkingParam._t + robot.walkingParam.T_s/2,  robot.walkingParam);
			}
			else{
				bezier_planning(&robot.rightLeg.x_ref,&robot.rightLeg.y_ref, robot.walkingParam._t-robot.walkingParam.T_s/2, robot.walkingParam);
			}

			if(robot.walkingParam._t>=robot.walkingParam.T_s){
				robot.walkingParam._t=0;
			}
			robot.walkingParam._t+=0.005;
			robot_acting();
			break;
		}
	}
	osDelay(5);
  }
  /* USER CODE END robot_task */
}

/*!
 * Compute robot position and pose by reading sensors
 *
 * @param void
 */
void robot_sensing()
{

	float phi_left_1, phi_left_2;
	float phi_right_1, phi_right_2;
	//电机角度补偿计算
	phi_left_1 = leftHip_Motor.assemble_bias - leftHip_Motor.curr_position;
	phi_left_2 = leftKnee_Motor.assemble_bias - leftKnee_Motor.curr_position;
	phi_right_1 = rightHip_Motor.curr_position + rightHip_Motor.assemble_bias;
	phi_right_2 = rightKnee_Motor.curr_position + rightKnee_Motor.assemble_bias;

	leg_forward_kinematics(&robot.leftLeg, LENGTH_HIP_LINK, LENGTH_KNEE_LINK, phi_left_1, phi_left_2);
	leg_forward_kinematics(&robot.rightLeg, LENGTH_HIP_LINK, LENGTH_KNEE_LINK, phi_right_1, phi_right_2);


	//left
	float* f_left = contact_force_estimate(phi_left_1 + M_PI_2, \
											phi_left_1 + phi_left_2, \
											-leftHip_Motor.curr_torque, leftKnee_Motor.curr_torque, \
											LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	robot.leftLeg.fx = f_left[X];
	robot.leftLeg.fy = f_left[Y];

	//right
	float* f_right = contact_force_estimate(phi_right_1 + M_PI_2, \
										phi_right_1 + phi_right_2, \
										rightHip_Motor.curr_torque, -rightKnee_Motor.curr_torque, \
										LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);

	robot.rightLeg.fx = f_right[X];
	robot.rightLeg.fy = f_right[Y];

}

void robot_acting()
{
	//robot_leg_admittance_ctrller();

	//注意调用完左腿ik后立即使用ang_left,否则会被右腿ik覆盖
	float* ang_left = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.leftLeg.x_ref, robot.leftLeg.y_ref);
	leftHip_Motor.ref_position = -ang_left[0] + leftHip_Motor.assemble_bias;
	leftKnee_Motor.ref_position = -ang_left[1] + leftKnee_Motor.assemble_bias ;


	float* ang_right = inverse_kinematics(LENGTH_HIP_LINK, LENGTH_KNEE_LINK, robot.rightLeg.x_ref, robot.rightLeg.y_ref);
	rightHip_Motor.ref_position = ang_right[0] - rightHip_Motor.assemble_bias;
	rightKnee_Motor.ref_position = ang_right[1] - rightKnee_Motor.assemble_bias;

	//robot_impedance_ctrller();
	robot_admittance_ctrller();

		leftHip_Motor.ref_cmd = sature(leftHip_Motor.ref_cmd, RIGHT_HIP_MAX_ANGLE, RIGHT_HIP_MIN_ANGLE);
		tmotor_set_position(LEFT_HIP_MOTOR_ID, leftHip_Motor.ref_cmd, leftHip_Motor.Kp_theta);
		//osDelay(1);
		leftKnee_Motor.ref_cmd = sature(leftKnee_Motor.ref_cmd, RIGHT_KNEE_MAX_ANGLE, RIGHT_KNEE_MIN_ANGLE);
		tmotor_set_position(LEFT_KNEE_MOTOR_ID, leftKnee_Motor.ref_cmd, leftKnee_Motor.Kp_theta);

//	leftHip_Motor.ref_position = sature(leftHip_Motor.ref_position, RIGHT_HIP_MAX_ANGLE, RIGHT_HIP_MIN_ANGLE);
//	tmotor_set_position(LEFT_HIP_MOTOR_ID, leftHip_Motor.ref_position, leftHip_Motor.Kp_theta);
//	//osDelay(1);
//	leftKnee_Motor.ref_position = sature(leftKnee_Motor.ref_position, RIGHT_KNEE_MAX_ANGLE, RIGHT_KNEE_MIN_ANGLE);
//	tmotor_set_position(LEFT_KNEE_MOTOR_ID, leftKnee_Motor.ref_position, leftKnee_Motor.Kp_theta);



//	//osDelay(1);
//	rightHip_Motor.ref_position = sature(rightHip_Motor.ref_position, LEFT_HIP_MAX_ANGLE, LEFT_HIP_MIN_ANGLE);
//	tmotor_set_position(RIGHT_HIP_MOTOR_ID, rightHip_Motor.ref_position, rightHip_Motor.Kp_theta);
//	//osDelay(1);
//	rightKnee_Motor.ref_position = sature(rightKnee_Motor.ref_position, LEFT_KNEE_MAX_ANGLE, LEFT_KNEE_MIN_ANGLE);
//	tmotor_set_position(RIGHT_KNEE_MOTOR_ID, rightKnee_Motor.ref_position, rightKnee_Motor.Kp_theta);

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

void robot_admittance_ctrller(void)
{
	admittance_calc(&admt_right_hip_diff, rightHip_Motor.average_torque, rightHip_Motor.ref_position);
	rightHip_Motor.ref_cmd = rightHip_Motor.ref_position - admt_right_hip_diff.ed;

	admittance_calc(&admt_right_knee_diff, rightKnee_Motor.average_torque, rightKnee_Motor.ref_position);
	rightKnee_Motor.ref_cmd = rightKnee_Motor.ref_position - admt_right_knee_diff.ed;

	admittance_calc(&admt_left_hip_diff, leftHip_Motor.average_torque, leftHip_Motor.ref_position);
	leftHip_Motor.ref_cmd = leftHip_Motor.ref_position - admt_left_hip_diff.ed;

	admittance_calc(&admt_left_knee_diff, leftKnee_Motor.average_torque, leftKnee_Motor.ref_position);
	leftKnee_Motor.ref_cmd = leftKnee_Motor.ref_position - admt_left_knee_diff.ed;

}
void robot_leg_admittance_ctrller(void)
{
	admittance_calc(&admt_left_x_diff, robot.leftLeg.fx, robot.leftLeg.x_ref);
	robot.leftLeg.x_cmd = robot.leftLeg.x_ref-admt_left_x_diff.ed;

	admittance_calc(&admt_left_y_diff, robot.leftLeg.fy, robot.leftLeg.y_ref);
	robot.leftLeg.y_cmd = robot.leftLeg.y_ref-admt_left_y_diff.ed;

	admittance_calc(&admt_right_x_diff, robot.rightLeg.fx, robot.rightLeg.x_ref);
	robot.rightLeg.x_cmd = robot.rightLeg.x_ref-admt_right_x_diff.ed;

	admittance_calc(&admt_right_y_diff, robot.rightLeg.fy, robot.rightLeg.y_ref);
	robot.rightLeg.y_cmd = robot.rightLeg.y_ref-admt_right_y_diff.ed;
}
