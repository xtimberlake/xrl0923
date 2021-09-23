/**
  * @file     communicationg_task.c
  * @version  0.5
  * @date     August,5th 2021
  * @brief    communicating task between Ubuntu and STM32, including transmit and receipt
  * 
  *	@author   Haoyun Yan
  */
  
#include "communicating_task.h"

#define  __RELAY_TASK_GLOBALS

chassis_t chassis;

union UNION_Float tx_data1;
union UNION_Float tx_data2;
uint8_t uart_tx_buff[8];

void communicatingTask(void *argument)
{
  /* USER CODE BEGIN communicatingTask */
//	osEvent event;
	tx_data1.f = 12.34;
	tx_data2.f = 56.78;

	uart_tx_buff[0] = tx_data1.ch[0];
	uart_tx_buff[1] = tx_data1.ch[1];
	uart_tx_buff[2] = tx_data1.ch[2];
	uart_tx_buff[3] = tx_data1.ch[3];

	uart_tx_buff[4] = tx_data2.ch[0];
	uart_tx_buff[5] = tx_data2.ch[1];
	uart_tx_buff[6] = tx_data2.ch[2];
	uart_tx_buff[7] = tx_data2.ch[3];
	//转换成ch类型

  /* Infinite loop */
  for(;;)
  {
	  //与下列函数配套使用
	  //osSignalSet(communicTaskHandle, CMD_SEND_FLAG);
//	  event = osSignalWait(CMD_SEND_FLAG, osWaitForever);
//
//	  		if (event.status == osEventSignal)
//	      {
//				if (event.value.signals & CMD_SEND_FLAG)
//				{//send_chassis_motor_ctrl_message(motor_cur.chassis_cur);
//
//				}
//
//
//	      }

	  chassis.vw += 0.1;
	  chassis.vx = sin(chassis.vw);

	  HAL_UART_Transmit(&huart6, uart_tx_buff, sizeof(uart_tx_buff), 0xFF);

    osDelay(1000);
  }
  /* USER CODE END communicatingTask */
}

void ubuntu_receive_callback(uint8_t *rx_buff, uint16_t length)
{
	;

}
