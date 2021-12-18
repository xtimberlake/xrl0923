#include "bsp_usart.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;
//extern UART_HandleTypeDef huart8;
//extern DMA_HandleTypeDef hdma_uart8_rx;


uint8_t dma_ubuntu_buff[DMA_UBUNTU_LEN];
//uint8_t dma_pc_cmd_buff[PC_CMD_LEN];
uint8_t dma_sixaxis_buff[DMA_SIXAXIS_LEN];
uint8_t dma_imu_buf[DMA_IMU_LEN];


/**
* @brief 串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
*/
void uart_pkg_init()
{
	__HAL_UART_CLEAR_IDLEFLAG(&IMU_HUART);
	__HAL_UART_ENABLE_IT(&IMU_HUART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&IMU_HUART, dma_imu_buf, DMA_IMU_LEN);
	
	__HAL_UART_CLEAR_IDLEFLAG(&UBUNTU_USART);
	__HAL_UART_ENABLE_IT(&UBUNTU_USART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&UBUNTU_USART, dma_ubuntu_buff, DMA_UBUNTU_LEN);

	__HAL_UART_CLEAR_IDLEFLAG(&SIXAXIS_USART);
	__HAL_UART_ENABLE_IT(&SIXAXIS_USART, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&SIXAXIS_USART, dma_sixaxis_buff, DMA_SIXAXIS_LEN);

}

/**
  * @brief 串口空闲中断   注：需在it.c中每个串口的中断中调用该函数
  * @param UART_HandleTypeDef *huart
  */
void user_uart_IRQHandle(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //判断是否是空闲中断
	{
	
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
		HAL_UART_DMAStop(huart);															//停止本次DMA运输
		user_uart_IDLECallback(huart);                        //调用串口功能回调函数
	}

}



/**
  * @brief 串口空闲中断回调函数
  * @param UART_HandleTypeDef *huart
  */
void user_uart_IDLECallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance== USART1)			//DBUS串口
//	{
//		/*process your data here*/
//		rc_callback_handler(&rc,dbus_buf);
//		//
//		
//		/*Re active DMA*/
//		HAL_UART_Receive_DMA(huart, dbus_buf, DBUS_BUFLEN);
//	}
	if (huart->Instance == USART6) {
		uint16_t length_data = 0;
		length_data = DMA_UBUNTU_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
		ubuntu_receive_callback(dma_ubuntu_buff, length_data);
		HAL_UART_Receive_DMA(huart, dma_ubuntu_buff, DMA_UBUNTU_LEN);
	}
	if (huart->Instance == UART8) {
		uint16_t length_data = 0;
		length_data = DMA_SIXAXIS_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx);
		sixaxis_callback_handle(dma_sixaxis_buff, length_data);
		HAL_UART_Receive_DMA(huart, dma_sixaxis_buff, DMA_SIXAXIS_LEN);
//	switch(dma_pc_cmd_buff[0]){
//		case 'i':
//		{
//			printf("Initial\r\n");
//			robot.state=0;
//			break;
//		}
//		case 'w':
//		{
//			printf("Work\r\n");
//			robot.state=1;
//			break;
//		}
//		case 't':
//		{
//			printf("trot\r\n");
//			robot.state=2;
//			break;
//		}
//		default:
//		{
//			break;
//		}
//	}


	}

	if(huart->Instance== UART7)
	{
			uint16_t length_data = 0;
			length_data = DMA_IMU_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart7_rx);
			xsens_callback_handle(dma_imu_buf,length_data);

			HAL_UART_Receive_DMA(huart, dma_imu_buf, DMA_IMU_LEN);
	}
}

//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
//#endif /* __GNUC__ */
//
//
//PUTCHAR_PROTOTYPE
//{
//
//    HAL_UART_Transmit(&PRINTF_USART, (uint8_t *)&ch, 1, 0xFFFF);  //调用STM32的HAL库，发送一个字节
//
//  return (ch);
//}


