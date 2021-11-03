#include "robot_sense.h"


void robot_sensingTask(void *argument)
{
  /* USER CODE BEGIN robot_sensingTask */
	uint32_t t;
  /* Infinite loop */
  for(;;)
  {
	  t = HAL_GetTick();
	  robot_sensing(t);
  }
  /* USER CODE END robot_sensingTask */
}

/*!
 * Compute robot position and pose by reading sensors
 *
 * @param void
 */
void robot_sensing(uint32_t t)
{
	static uint32_t time_interval; // unit: ms
	robot.sensing_t[NOW] = t;

	time_interval = robot.sensing_t[NOW] - robot.sensing_t[LAST];


	float phi_left_1, phi_left_2;
	float phi_right_1, phi_right_2;
	//电机角度补偿计算
	phi_left_1 = leftHip_Motor.assemble_bias - leftHip_Motor.curr_position;
	phi_left_2 = leftKnee_Motor.assemble_bias - leftKnee_Motor.curr_position;
	phi_right_1 = rightHip_Motor.curr_position + rightHip_Motor.assemble_bias;
	phi_right_2 = rightKnee_Motor.curr_position + rightKnee_Motor.assemble_bias;

	leg_forward_kinematics(&robot.leftLeg, LENGTH_HIP_LINK, LENGTH_KNEE_LINK, phi_left_1, phi_left_2);
	leg_forward_kinematics(&robot.rightLeg, LENGTH_HIP_LINK, LENGTH_KNEE_LINK, phi_right_1, phi_right_2);

	if(time_interval >= 10)
		{
			robot.leftLeg.dx = (robot.leftLeg.x - robot.leftLeg.last_x)*1000/time_interval;
			robot.leftLeg.dy = (robot.leftLeg.y - robot.leftLeg.last_y)*1000/time_interval;
			robot.rightLeg.dx = (robot.rightLeg.x - robot.rightLeg.last_x)*1000/time_interval;
			robot.rightLeg.dy = (robot.rightLeg.y - robot.rightLeg.last_y)*1000/time_interval;

			robot.leftLeg.last_x = robot.leftLeg.x;
			robot.leftLeg.last_y = robot.leftLeg.y;
			robot.rightLeg.last_x = robot.rightLeg.x;
			robot.rightLeg.last_y = robot.rightLeg.y;
			robot.sensing_t[LAST] = robot.sensing_t[NOW];
		}
		else {
			;
		}


	//left
	float* f_left = contact_force_estimate(phi_left_1 + M_PI_2, \
											phi_left_1 + phi_left_2, \
											-leftHip_Motor.average_torque, leftKnee_Motor.average_torque, \
											LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);
	robot.leftLeg.fx = -f_left[X];
	robot.leftLeg.fy = -f_left[Y];

	//right
	float* f_right = contact_force_estimate(phi_right_1 + M_PI_2, \
										phi_right_1 + phi_right_2, \
										rightHip_Motor.average_torque, -rightKnee_Motor.average_torque, \
										LENGTH_HIP_LINK/1000, LENGTH_KNEE_LINK/1000);

	robot.rightLeg.fx = -f_right[X];
	robot.rightLeg.fy = -f_right[Y];

}
