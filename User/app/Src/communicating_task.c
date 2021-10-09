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

int test_int = 10;
chassis_t chassis;

union UNION_Float tx_data1;
union UNION_Float tx_data2;
uint8_t uart_tx_buff[8];
uint8_t ubuntu_uart_tx_buff[8];

uint8_t* from_float_to_uint(float f)
{
	static uint8_t  u8_data[4];
	union UNION_Float union_buff;
	union_buff.f = f;

	u8_data[0] = union_buff.ch[0];
	u8_data[1] = union_buff.ch[1];
	u8_data[2] = union_buff.ch[2];
	u8_data[3] = union_buff.ch[3];

	return u8_data;
}

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


	float sin_varible;
	float cos_varible;

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

	  chassis.vw += chassis.vy;
	  chassis.vx = 50 * sin(chassis.vw);

	  sin_varible = sin(chassis.vw);
	  cos_varible = cos(chassis.vw) + 1.5;

	  uint8_t* tf_buff = from_float_to_uint(sin_varible);
	  for (int i = 0; i < 4; i++) {
		  ubuntu_uart_tx_buff[i] = tf_buff[i];
	  }
	  uint8_t* tf_buff_2 = from_float_to_uint(cos_varible);
	  for (int i = 4; i < 8; i++) {
		  ubuntu_uart_tx_buff[i] = tf_buff_2[i-4];
	  }
	  test_int += 1;
	  HAL_UART_Transmit(&huart6, ubuntu_uart_tx_buff, sizeof(ubuntu_uart_tx_buff), 0xFF);
	  //HAL_UART_Transmit(&huart6, uart_tx_buff, sizeof(uart_tx_buff), 0xFF);

    osDelay(100);
  }
  /* USER CODE END communicatingTask */
}

void ubuntu_receive_callback(uint8_t *rx_buff, uint16_t length)
{
	;

}
