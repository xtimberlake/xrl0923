/**
  * @file tmotor_bsp.c
  * @version 0.5
  * @date April,17 2021
  * @brief  MIT Cheetah CAN command file
	* 
	*
	* reference: 
		<Hobbyking Cheetah Compact DRV8323> 
		link: https://os.mbed.com/users/benkatz/code/Hobbyking_Cheetah_Compact_DRV8323//file/6cd89bd6fcaa/CAN/CAN_com.cpp/
	*
  *	@author Haoyun Yan
  */
	
#include "bsp_tmotor.h"


#define __TMOTOR_BSP_GLOBALS


CAN_TxHeaderTypeDef tmotor_mode_tx_handle; 
uint8_t             tmotor_mode_tx_data[8];

CAN_TxHeaderTypeDef tmotor_control_tx_handle; 
uint8_t             tmotor_control_tx_data[8];

CAN_TxHeaderTypeDef tmotor_serial_control_tx_handle;
uint8_t             tmotor_serial_control_tx_data[8];

tmotor_handle_t leftHip_Motor;
tmotor_handle_t leftKnee_Motor;
tmotor_handle_t rightHip_Motor;
tmotor_handle_t rightKnee_Motor;
tmotor_handle_t test_Motor;

motor_parms_t m;

void tmotor_init(tmotor_handle_t* m, uint8_t id,\
				  float assemble_bias, \
				  float speed_max, \
				  float Kp_theta, float Kd_theta,\
				  float Kp_v, float Ki_v, float Kd_v)
{
	m->id =id;
	m->assemble_bias = assemble_bias;
	m->speed_max = speed_max;
	m->Kp_theta = Kp_theta;
	m->Kd_theta = Kd_theta;
	m->Kp_v = Kp_v;
	m->Ki_v = Ki_v;
	m->Kd_v = Kd_v;

	TMotor_setMode(m->id, TMotor_Set_Zore);
	osDelay(10);
	TMotor_setMode(m->id, TMotor_Set_Zore);
	osDelay(10);
	TMotor_setMode(m->id, TMotor_Reset_Mode);


}

void pkg_tmotor_init()
{

	m.h_kp_theta = 55.0f;
	m.h_kd_theta = 10.0f;
	m.h_kp_v = 80.0f;
	m.h_ki_v = 0.0f;
	m.h_kd_v = 0.0f;

	m.k_kp_theta = 55.0f;
	m.k_kd_theta = 10.0f;
	m.k_kp_v = 80.0f;
	m.k_ki_v = 0.0f;
	m.k_kd_v = 0.0f;

	tmotor_init(&leftHip_Motor, LEFT_HIP_MOTOR_ID, \
				HIP_MOTOR_OFFSET, \
				9.5, \
				m.h_kp_theta, m.h_kd_theta, \
				m.h_kp_v, m.h_ki_v, m.h_kd_v);
	osDelay(5);
	tmotor_init(&leftKnee_Motor, LEFT_KNEE_MOTOR_ID, \
				KNEE_MOTOR_OFFSET, \
				9.5, \
				m.k_kp_theta, m.k_kd_theta, \
				m.k_kp_v, m.k_ki_v, m.k_kd_v);
	osDelay(5);
	tmotor_init(&rightHip_Motor, RIGHT_HIP_MOTOR_ID, \
			    HIP_MOTOR_OFFSET, \
				9.5, \
				m.h_kp_theta, m.h_kd_theta, \
				m.h_kp_v, m.h_ki_v, m.h_kd_v);
	osDelay(5);
	tmotor_init(&rightKnee_Motor, RIGHT_KNEE_MOTOR_ID, \
				KNEE_MOTOR_OFFSET, \
				9.5, \
				m.k_kp_theta, m.k_kd_theta, \
				m.k_kp_v, m.k_ki_v, m.k_kd_v);
	osDelay(5);

}

void TMotor_setMode(HipMotorID_TypeDef tID, TMotor_Mode_TypeDef tMode)
{
	uint8_t FreeTxNum = 0;  
	uint32_t send_mail_box;
	tmotor_mode_tx_handle.StdId = tID; 
	tmotor_mode_tx_handle.IDE = CAN_ID_STD;
	tmotor_mode_tx_handle.RTR = CAN_RTR_DATA;
	tmotor_mode_tx_handle.DLC = 0x08;
	tmotor_mode_tx_data[0] = 0xFF; //命令字
	tmotor_mode_tx_data[1] = 0xFF;		//Node-ID
	tmotor_mode_tx_data[2] = 0xFF;
	tmotor_mode_tx_data[3] = 0xFF;
	tmotor_mode_tx_data[4] = 0xFF;
	tmotor_mode_tx_data[5] = 0xFF;
	tmotor_mode_tx_data[6] = 0xFF;
	tmotor_mode_tx_data[7] = tMode;
	
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&TMotor_CAN);  
	while(FreeTxNum == 0) 
	{  	
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&TMotor_CAN);  
  }
	
	while(HAL_CAN_AddTxMessage(&TMotor_CAN, &tmotor_mode_tx_handle, tmotor_mode_tx_data, &send_mail_box) != HAL_OK)
	{
		Error_Handler();
	}

}

/**
  * @brief     left T-Motor position + velocity control 
  * @param     tff: desired torque

  */
void leftHipMotor_setPosition(float p_des, float kp)
{
	//tmotor_pkg_cmd(LeftHipMotor_ID, p_des, 0, kp, 0, 0);

}

/**
  * @brief     right T-Motor position + velocity control 
  * @param     tff: desired torque

  */
void rightHipMotor_setPosition(float p_des, float kp)
{
	//tmotor_pkg_cmd(RightHipMotor_ID, p_des, 0, kp, 0, 0);
}

void tmotor_set_position(uint8_t id, float p_des, float kp)
{

	switch(id)
	{
	case LEFT_HIP_MOTOR_ID:
		{tmotor_serial_mode_send_cmd(id, p_des, leftHip_Motor.speed_max, kp,\
				leftHip_Motor.Kd_theta, leftHip_Motor.Kp_v, leftHip_Motor.Kd_v, leftHip_Motor.Ki_v);
		calculatee_ref_radps(&leftHip_Motor);
		break;}
	case LEFT_KNEE_MOTOR_ID:
		{tmotor_serial_mode_send_cmd(id, p_des, leftKnee_Motor.speed_max, kp,\
				leftKnee_Motor.Kd_theta, leftKnee_Motor.Kp_v, leftKnee_Motor.Kd_v, leftKnee_Motor.Ki_v);
		calculatee_ref_radps(&leftKnee_Motor);
		break;}
	case RIGHT_HIP_MOTOR_ID:
		{tmotor_serial_mode_send_cmd(id, p_des, rightHip_Motor.speed_max, kp,\
				rightHip_Motor.Kd_theta, rightHip_Motor.Kp_v, rightHip_Motor.Kd_v, rightHip_Motor.Ki_v);
		calculatee_ref_radps(&rightHip_Motor);
		break;}
	case RIGHT_KNEE_MOTOR_ID:
		{tmotor_serial_mode_send_cmd(id, p_des, rightKnee_Motor.speed_max, kp,\
				rightKnee_Motor.Kd_theta, rightKnee_Motor.Kp_v, rightKnee_Motor.Kd_v, rightKnee_Motor.Ki_v);
		calculatee_ref_radps(&rightKnee_Motor);
		break;}
	}

}

void test_motor_set_position(float p_des, float kp)
{
	test_Motor.ref_position = p_des;
	test_Motor.Kp_theta = kp;
	tmotor_serial_mode_send_cmd(test_ID, p_des, test_Motor.speed_max, kp,\
						test_Motor.Kd_theta, test_Motor.Kp_v, test_Motor.Kd_v, test_Motor.Ki_v);

}

void calculatee_ref_radps(tmotor_handle_t* motor)
{
	motor->pos_err[NOW] = (motor->ref_position - motor->curr_position);
	motor->dpos_err = motor->pos_err[NOW] - motor->pos_err[LAST];
	motor->calc_speed_radps = 0.1 * motor->Kp_theta * (motor->ref_position - motor->curr_position) \
								+ 0.1 * motor->Kp_v * (motor->dpos_err);

	motor->pos_err[LAST] = motor->pos_err[NOW];
}

/**
  * @brief     力位模式控制下:T-Motor 数据发送
  * @param  
  */
void tmotor_pkg_cmd(HipMotorID_TypeDef tID,float p_des, float v_des, float kp, float kd, float t_ff)
{
	
	uint8_t FreeTxNum = 0;  
	uint32_t send_mail_box;
	
	// limit data to be within bounds
	p_des = fminf(fmaxf(position_MIN,p_des), position_MAX);
	v_des = fminf(fmaxf(velocity_MIN,v_des), velocity_MAX);
	kp = fminf(fmaxf(KP_MIN,kp), KP_MAX);
	kd = fminf(fmaxf(KD_MIN,kd), KD_MAX);
	t_ff = fminf(fmaxf(Torque_MIN,t_ff), Torque_MAX);
	
	//convert floats to unsigned ints
	int p_int = float_to_uint(p_des, position_MIN, position_MAX, 16);
	int v_int = float_to_uint(v_des, velocity_MIN, velocity_MAX, 12);
	int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	int t_int = float_to_uint(t_ff, Torque_MIN, Torque_MAX, 12);
	
	//pack ints into the can buffer
	tmotor_control_tx_data[0] = p_int >> 8;   
	tmotor_control_tx_data[1] = p_int & 0xFF; 
	tmotor_control_tx_data[2] = v_int >> 4;
	tmotor_control_tx_data[3] = ((v_int&0xF)<<4)|(kp_int>>8);
	tmotor_control_tx_data[4] = kp_int & 0xFF;
	tmotor_control_tx_data[5] = kd_int >> 4;
	tmotor_control_tx_data[6] = ((kd_int&0xF)<<4)|(t_int>>8);
	tmotor_control_tx_data[7] = t_int & 0xFF;
	

	tmotor_control_tx_handle.StdId = tID; 
	tmotor_control_tx_handle.IDE = CAN_ID_STD;
	tmotor_control_tx_handle.RTR = CAN_RTR_DATA;
	tmotor_control_tx_handle.DLC = 0x08;
	
		FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&TMotor_CAN);  
	while(FreeTxNum == 0) 
	{  	
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&TMotor_CAN);  
  }
	
	while(HAL_CAN_AddTxMessage(&TMotor_CAN, &tmotor_control_tx_handle, tmotor_control_tx_data, &send_mail_box) != HAL_OK)
	{
		Error_Handler();
	}
	
}

/**
  * @brief     串级控制下:T-Motor 数据发送
  * @param
  */
void tmotor_serial_mode_send_cmd(HipMotorID_TypeDef tID,float p_des, float v_des, \
								 float kp_p, float kd_p, \
								 float kp_v, float kd_v, float ki_v)
{

//	p_des = fminf(fmaxf(position_MIN,p_des), position_MAX);
//	v_des = fminf(fmaxf(velocity_MIN,v_des), velocity_MAX);
//	kp_p = fminf(fmaxf(KP_MIN,kp_p), KP_MAX);
//	kd_p = fminf(fmaxf(KD_MIN,kd_p), KD_MAX);
//	kp_v = fminf(fmaxf(KP_MIN,kp_v), KP_MAX);
//	kd_v = fminf(fmaxf(KD_MIN,kd_v), KD_MAX);
//	ki_v = fminf(fmaxf(KI_MIN,ki_v), KI_MAX);


	int p = float_to_uint(p_des, position_MIN, position_MAX, 16);
	int v = float_to_uint(v_des, velocity_MIN, velocity_MAX, 8);
	int kp = float_to_uint(kp_p, KP_MIN, KP_MAX, 8);
	int kd = float_to_uint(kd_p, KD_MIN, KD_MAX, 8);
	int kp_i = float_to_uint(kp_v,KP_MIN,KP_MAX,8);
	int kd_i = float_to_uint(kd_v,KD_MIN,KD_MAX,8);
	int ki_i = float_to_uint(ki_v,KI_MIN,KI_MAX,8);


	tmotor_serial_control_tx_data[0] = p >> 8;
	tmotor_serial_control_tx_data[1] = p & 0xFF;
	tmotor_serial_control_tx_data[2] = v;
	tmotor_serial_control_tx_data[3] = kp;
	tmotor_serial_control_tx_data[4] = kd;
	tmotor_serial_control_tx_data[5] = kp_i;
	tmotor_serial_control_tx_data[6] = kd_i;
	tmotor_serial_control_tx_data[7] = ki_i;

	uint8_t FreeTxNum = 0;
	uint32_t send_mail_box;
	FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&TMotor_CAN);

	tmotor_serial_control_tx_handle.StdId = tID;
	tmotor_serial_control_tx_handle.IDE = CAN_ID_STD;
	tmotor_serial_control_tx_handle.RTR = CAN_RTR_DATA;
	tmotor_serial_control_tx_handle.DLC = 0x08;

	while(FreeTxNum == 0)
	{
	    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&TMotor_CAN);
	}

	while(HAL_CAN_AddTxMessage(&TMotor_CAN, &tmotor_serial_control_tx_handle, tmotor_serial_control_tx_data, &send_mail_box) != HAL_OK)
	{
		Error_Handler();
	}

}

void motor_param_change()
{
	leftHip_Motor.Kp_theta = m.h_kp_theta;
	leftHip_Motor.Kd_theta = m.h_kd_theta;
	leftHip_Motor.Kp_v = m.h_kp_v;
	leftHip_Motor.Ki_v = m.h_ki_v;
	leftHip_Motor.Kd_v = m.h_kd_v;

	leftKnee_Motor.Kp_theta = m.k_kp_theta;
	leftKnee_Motor.Kd_theta = m.k_kd_theta;
	leftKnee_Motor.Kp_v = m.k_kp_v;
	leftKnee_Motor.Ki_v = m.k_ki_v;
	leftKnee_Motor.Kd_v = m.k_kd_v;

	rightHip_Motor.Kp_theta = m.h_kp_theta;
	rightHip_Motor.Kd_theta = m.h_kd_theta;
	rightHip_Motor.Kp_v = m.h_kp_v;
	rightHip_Motor.Ki_v = m.h_ki_v;
	rightHip_Motor.Kd_v = m.h_kd_v;

	rightKnee_Motor.Kp_theta = m.k_kp_theta;
	rightKnee_Motor.Kd_theta = m.k_kd_theta;
	rightKnee_Motor.Kp_v = m.k_kp_v;
	rightKnee_Motor.Ki_v = m.k_ki_v;
	rightKnee_Motor.Kd_v = m.k_kd_v;

}

void unpackCanInfoFromMotor(uint8_t* data, tmotor_handle_t* motor)
{
	int id, p_int=0x00, v_int=0x00, t_int=0x00;
	id = data[0];
	if(motor->id == id)
	{
	p_int = (data[1] << 8)| (0x00ff&data[2]);
	v_int = (data[3] << 4) | ((0x00ff&data[4]) >> 4);
	t_int = ((data[4] & 0xF) << 8) | (0x00ff&data[5]);
	p_int &= 0x0000ffff; //16bit
	v_int &= 0x00000fff; //12bit
	t_int &= 0x00000fff; //12bit


	motor->curr_position = uint_to_float(p_int,position_MIN,position_MAX,16);
	motor->curr_speed_radps = 10*uint_to_float(v_int,velocity_MIN,velocity_MAX,12);
	motor->curr_torque = uint_to_float(t_int,Torque_MIN,Torque_MAX,12);
	motor->fault = (data[6] << 8) | data[7];

	if(motor->buff_cnt >= SMOOTH_NUM){
				motor->buff_cnt = 0;
			}
	motor->torque_buff[motor->buff_cnt] = motor->curr_torque;
	motor->buff_cnt++;
	float sum;
	for (int i = 0; i < SMOOTH_NUM; i++) {
		sum += motor->torque_buff[i];
	}
	motor->average_torque = sum / SMOOTH_NUM;



	}
}

//mathematically transfer
float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }


float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }


