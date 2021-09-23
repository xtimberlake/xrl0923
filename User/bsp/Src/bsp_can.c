#include "bsp_can.h"


#define test_tx_can hcan1
CAN_RxHeaderTypeDef rx_header;




void can_filter_init()
{
	//滤波器只接受TPDO数据
	
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterActivation = ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
	{	Error_Handler();
	//printf("CAN1 ConfigFilter Failed!\n");
	}
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{	Error_Handler();
	//printf("CAN1 START Failed!\n");
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{	Error_Handler();
	//printf("CAN1 ActivateNotification Failed!\n");}
	}
	
//	can_filter_st.SlaveStartFilterBank = 14;
//	can_filter_st.FilterBank = 14;
//	if(HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
//	{	Error_Handler(); printf("CAN2 ConfigFilter Failed!\n");}
//  if(HAL_CAN_Start(&hcan2) != HAL_OK )
//	{	Error_Handler(); printf("CAN2 START Failed!\n");}
//  if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK )
//	{	Error_Handler(); printf("CAN2 ActivateNotification Failed!\n");}
	
	
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
	uint8_t rx_data[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	if(hcan->Instance == hcan1.Instance)
	{
	switch(rx_data[0])
	{
	case test_ID:
		{unpackCanInfoFromMotor(rx_data, &test_Motor); break;}
	case LEFT_HIP_MOTOR_ID:
		{unpackCanInfoFromMotor(rx_data, &leftHip_Motor); break;}
	case LEFT_KNEE_MOTOR_ID:
		{unpackCanInfoFromMotor(rx_data, &leftKnee_Motor); break;}
	case RIGHT_HIP_MOTOR_ID:
		{unpackCanInfoFromMotor(rx_data, &rightHip_Motor); break;}
	case RIGHT_KNEE_MOTOR_ID:
		{unpackCanInfoFromMotor(rx_data, &rightKnee_Motor); break;}
	default: break;
	}

	}
	
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

