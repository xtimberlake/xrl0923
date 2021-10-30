#include "tmotor_cmd_task.h"

float temp_right_hip_motor_torque = 0;
float temp_right_knee_motor_torque = 0;



void tmotorTask(void *argument)
{
  /* USER CODE BEGIN tmotorTask */


  /* Infinite loop */
  for(;;)
  {
	  temp_right_hip_motor_torque = rightHip_Motor.curr_torque;
	  temp_right_knee_motor_torque = rightKnee_Motor.curr_torque;

	  if(robot.state == INITIALIZING)
	  {
		  robot.last_state = INITIALIZING;
		  TMotor_setMode(LEFT_HIP_MOTOR_ID, TMotor_Reset_Mode);
		  osDelay(1);
		  TMotor_setMode(LEFT_KNEE_MOTOR_ID, TMotor_Reset_Mode);
		  osDelay(1);
		  TMotor_setMode(RIGHT_HIP_MOTOR_ID, TMotor_Reset_Mode);
		  osDelay(1);
		  TMotor_setMode(RIGHT_KNEE_MOTOR_ID, TMotor_Reset_Mode);
		  osDelay(2);
	  }
	  if(robot.state == WORKING && robot.last_state == INITIALIZING)
	  {

		  TMotor_setMode(LEFT_HIP_MOTOR_ID, TMotor_Control_Mode);

		  osDelay(1);
		  TMotor_setMode(LEFT_KNEE_MOTOR_ID, TMotor_Control_Mode);
		  osDelay(1);
		  TMotor_setMode(RIGHT_HIP_MOTOR_ID, TMotor_Control_Mode);
		  osDelay(1);
		  TMotor_setMode(RIGHT_KNEE_MOTOR_ID, TMotor_Control_Mode);
		  osDelay(2);
		  robot.last_state = WORKING;
	  }
	  if(robot.state == TROTING && robot.last_state == WORKING)
		  {
		  	  robot.walkingParam._t=0;
			  robot.last_state = TROTING;
		  }
	  if(robot.state == WORKING && robot.last_state == TROTING)
	  		  {
	  			  robot.last_state = WORKING;
	  		  }
	  osDelay(1);





  }
  /* USER CODE END tmotorTask */
}
